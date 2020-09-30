#include"semantic_mesh_loam/laser_odometry.h"


namespace semloam{

	LaserOdometry::LaserOdometry(void){

		scanperiod = 0.1;
		ioratio = 2;
		max_iterations = 25;
		delta_t_abort = 0.1;
		delta_r_abort = 0.1;

		
		velo_scans.reserve(scan_size);
		velo_scans_child.reserve(scan_size);

		CloudCentroid.reserve(feature_size);
		CloudEdge.reserve(feature_size);

		FeatureCloud.reserve(feature_size);
		FeatureCloud_child.reserve(feature_size);
		
		//_lastCloudCentroid.reserve(feature_size);
		//_lastCloudEdge.reserve(feature_size);
		_lastFeatureCloud.reserve(feature_size);

		//CloudCentroidInd.reserve(ind_size);
		//_lastCloudCentroidInd.reserve(ind_size);

		//CloudEdgeInd.reserve(ind_size);
		//_lastCloudEdgeInd.reserve(ind_size);

		tmp_pc_stored.reserve(feature_size);

		velo_scans.header.frame_id = "map";
		velo_scans_child.header.frame_id = "laserodometry";

		FeatureCloud.header.frame_id = "map";
		FeatureCloud_child.header.frame_id = "laserodometry";
		_lastFeatureCloud.header.frame_id = "map";

		odom_data.header.frame_id = "map";
		odom_data.child_frame_id = "vehicle";

		_last_odom_data.header.frame_id = "map";
		_last_odom_data.child_frame_id = "vehicle";

		laserodometry.header.frame_id = "map";
		laserodometry.child_frame_id = "laserodometry";

		laserodometry_to_map.frame_id_ = "map";
		laserodometry_to_map.child_frame_id_ = "laserodometry";

		system_initialized_checker = false;

	}

	bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode){

		float fparam;
		int iparam;

		std::cout << "Set up Laser Odometry" << std::endl;

		if(privateNode.getParam("systemdelay", iparam)){
			if(iparam < 1){
				ROS_ERROR("Invalid __systemdelay parameter: %d",iparam);
				return false;
			}
			else{
				__systemdelay = iparam;
			}
		}

		std::cout << "__systemdelay" << std::endl;

		if(privateNode.getParam("MaxCorrespondDistance", fparam)){
			if(fparam < 0.0){
				ROS_ERROR("Invalid MaxCorrespondDistance parameter: %f",fparam);
				return false;
			}
			else{
				MaxCorrespondDistance = fparam;
				std::cout << "MaxCorrespondDistance: " << MaxCorrespondDistance << std::endl;
			}
		}
		
		if(privateNode.getParam("MaximumIterations", iparam)){
			if(iparam < 0){
				ROS_ERROR("Invalid MaximumIterations parameter: %d",iparam);
				return false;
			}
			else{
				MaximumIterations = iparam;
			}
		}
		
		if(privateNode.getParam("TransformationEpsilon", fparam)){
			if(fparam < 0.0){
				ROS_ERROR("Invalid TransformationEpsilon parameter: %f",fparam);
				return false;
			}
			else{
				TransformationEpsilon = fparam;
			}
		}

		if(privateNode.getParam("EuclideanFitnessEpsilon", fparam)){
			if(fparam < 0.0){
				ROS_ERROR("Invalid EuclideanFitnessEpsilon parameter: %f",fparam);
				return false;
			}
			else{
				EuclideanFitnessEpsilon = fparam;
			}
		}

		if(privateNode.getParam("scanperiod", fparam)){
			if(fparam <= 0){
				ROS_ERROR("Invalid scanperiod parameter: %f",fparam);
				return false;
			}
			else{
				scanperiod = fparam;
			}
		}

		if(privateNode.getParam("ioratio", iparam)){
			if(iparam < 1){
				ROS_ERROR("Invalid ioratio parameter: %d", iparam);
				return false;
			}
			else{
				ioratio = iparam;
			}
		}

		if(privateNode.getParam("maxiterations", iparam)){
			if(iparam < 1){
				ROS_ERROR("Invalid maxiteration parameter: %d", iparam);
				return false;
			}
			else{
				max_iterations = iparam;
			}
		}

		if(privateNode.getParam("deltaTabort", fparam)){
			if(fparam <= 0.0){
				ROS_ERROR("Invalid deltaTabort parameter: %f", fparam);
				return false;
			}
			else{
				delta_t_abort = fparam;
			}
		}

		if(privateNode.getParam("deltaRabort", fparam)){
			if(fparam <= 0.0){
				ROS_ERROR("Invalid deltaRabort parameter: %f", fparam);
				return false;
			}
			else{
				delta_r_abort = fparam;
			}
		}

		std::cout << "private node set up end" << std::endl;

		_pubLaserOdomToInit = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
		_pubVelodynePoints3 = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points3", 2);
		_pubCentroidPointLast = node.advertise<sensor_msgs::PointCloud2>("/centroid_point_last", 2);
		_pubEdgePointLast = node.advertise<sensor_msgs::PointCloud2>("/edge_point_last", 2);

		//_pubFeaturePoints3 = node.advertise<sensor_msgs::PointCloud2>("/feature_points3", 2);

		//subscribe
		_subVelodynePoints = node.subscribe<sensor_msgs::PointCloud2>
			("/velodyne_points2", 2, &LaserOdometry::velodyne_callback, this);
		_subCentroid = node.subscribe<sensor_msgs::PointCloud2>
			("/centroid_points", 2, &LaserOdometry::centroid_callback, this);
		_subEdge = node.subscribe<sensor_msgs::PointCloud2>
			("/edge_points", 2, &LaserOdometry::edge_callback, this);
		_subOdometry = node.subscribe<nav_msgs::Odometry>
			("/odometry2", 2, &LaserOdometry::odometry_callback, this);
		
		_pub_init_slide_pose = node.advertise<geometry_msgs::PoseStamped>
			("/init_slide_pose", 10);

		std::cout << "End set up Laser Odometry" << std::endl;

		return true;
	}

	void LaserOdometry::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata){
		velo_scans_time = clouddata->header.stamp;

		//std::cout << "velo_scans_time: " << velo_scans_time << std::endl;
		
		pcl::fromROSMsg(*clouddata, velo_scans_child);
		velo_scans_child.header.frame_id = "laserodometry";

		//std::cout << "velo data catched" << std::endl;

		velo_scans_checker = true;
	}

	void LaserOdometry::centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata){
		CloudCentroid_time = centroiddata->header.stamp;

		pcl::fromROSMsg(*centroiddata, CloudCentroid);

		FeatureCloud_child = FeatureCloud_child + CloudCentroid;
		FeatureCloud_child.header.frame_id = "laserodometry";

		//std::cout << FeatureCloud_child.header.frame_id << std::endl;

		//std::cout << "catch centroid data" << std::endl;

		CloudCentroid_checker = true;
	}

	void LaserOdometry::edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata){
		CloudEdge_time = edgedata->header.stamp;

		pcl::fromROSMsg(*edgedata, CloudEdge);

		FeatureCloud_child = FeatureCloud_child + CloudEdge;
		FeatureCloud_child.header.frame_id = "laserodometry";

		//std::cout << "catch edge data" << std::endl;

		CloudEdge_checker = true;
	}

	void LaserOdometry::odometry_callback(const nav_msgs::OdometryConstPtr& odomdata){
		
		//_last_odom_data_time = odom_data_time;

		odom_data_time = odomdata->header.stamp;

		//std::cout << "      odom_data_time: " << odom_data_time << std::endl;
		//std::cout << "_last_odom_data_time: " << _last_odom_data_time << std::endl;


		odom_data = *odomdata;

		//std::cout << "odom_data frame id" << std::endl;
		//std::cout << odom_data.header.frame_id << std::endl;

		//std::cout << "catch odom data" << std::endl;

		odom_data_checker = true;
	}

	bool LaserOdometry::hasNewData(){

		bool checker_in = false;

		if(             velo_scans_checker == true
				&& CloudCentroid_checker == true
				&& CloudEdge_checker == true
				&& odom_data_checker == true)
		{	
			checker_in = true;
		}

		/*
		if(             (fabs(velo_scans_time - CloudCentroid_time) < 0.01)
				&& (fabs(velo_scans_time - CloudEdge_time) < 0.01)
				&& (fabs(velo_scans_time - odom_data_time) < 0.01))
		{
			checker_in = true;
		}
		else{
			checker_in = false;
		}*/

		return checker_in;
	}

	/*
	void LaserOdometry::get_relative_trans(){

		relative_pos_trans.dx = odom_data.pose.pose.position.x - _last_odom_data.pose.pose.position.x;
		relative_pos_trans.dy = odom_data.pose.pose.position.y - _last_odom_data.pose.pose.position.y;
		relative_pos_trans.dz = odom_data.pose.pose.position.z - _last_odom_data.pose.pose.position.z;

		tf::Quaternion now_quat, last_quat;

		quaternionMsgToTF(       odom_data.pose.pose.orientation,  now_quat );
		quaternionMsgToTF( _last_odom_data.pose.pose.orientation, last_quat );

		tf::Quaternion relative_quat = now_quat * inverse( last_quat );

		relative_quat.normalize();

		relative_pos_trans.dqx = relative_quat.x();
		relative_pos_trans.dqy = relative_quat.y();
		relative_pos_trans.dqz = relative_quat.z();
		relative_pos_trans.dqw = relative_quat.w();

		double dt = odom_data_time.toSec() - _last_odom_data_time.toSec();

		relative_pos_trans.vx = relative_pos_trans.dx / dt;
		relative_pos_trans.vy = relative_pos_trans.dy / dt;
		relative_pos_trans.vz = relative_pos_trans.dz / dt;

		double droll, dpitch, dyaw;

		tf::Matrix3x3( relative_quat ).getRPY( droll, dpitch, dyaw );

		relative_pos_trans.vqx = droll / dt;
		relative_pos_trans.vqy = dpitch/ dt;
		relative_pos_trans.vqz = dyaw  / dt;

	}*/
	
	void LaserOdometry::get_relative_trans(){

		
		relative_pos_trans.dx = odom_data.pose.pose.position.x - _last_odom_data.pose.pose.position.x;
		relative_pos_trans.dy = odom_data.pose.pose.position.y - _last_odom_data.pose.pose.position.y;
		relative_pos_trans.dz = odom_data.pose.pose.position.z - _last_odom_data.pose.pose.position.z;

		tf::Quaternion now_quat, last_quat;

		double dt = odom_data_time.toSec() - _last_odom_data_time.toSec();

		double now_roll, now_pitch, now_yaw;
		double last_roll, last_pitch, last_yaw;

		quaternionMsgToTF(       odom_data.pose.pose.orientation,  now_quat );
		quaternionMsgToTF( _last_odom_data.pose.pose.orientation, last_quat );

		tf::Matrix3x3( now_quat ).getRPY( now_roll , now_pitch , now_yaw );
		//std::cout << "now_roll: " << now_roll << std::endl;

		tf::Matrix3x3( last_quat ).getRPY( last_roll, last_pitch , last_yaw );
		//std::cout << "last_roll:" << last_roll << std::endl;

		double droll = now_roll - last_roll;
		double dpitch = now_pitch - last_pitch;
		double dyaw = now_yaw - last_yaw;
		
		//double droll = atan2( sin(now_roll - last_roll), cos(now_roll - last_roll) );
		//double dpitch = atan2( sin(now_pitch - last_pitch), cos(now_pitch - last_pitch));
		//double dyaw = atan2( sin(now_yaw - last_yaw) , cos(now_yaw - last_yaw) );

		relative_pos_trans.droll = droll;
		relative_pos_trans.dpitch = dpitch;
		relative_pos_trans.dyaw = dyaw;

		/*
		std::cout <<"droll" << droll << std::endl;
		std::cout <<"dpitch" << dpitch << std::endl;
		std::cout << "dyaw" << dyaw << std::endl;
		*/

		tf::Quaternion dquat = tf::createQuaternionFromRPY( droll , dpitch , dyaw );
		geometry_msgs::Quaternion geo_dquat;

		quaternionTFToMsg( dquat , geo_dquat );

		relative_pos_trans.dqx = geo_dquat.x;
		relative_pos_trans.dqy = geo_dquat.y;
		relative_pos_trans.dqz = geo_dquat.z;
		relative_pos_trans.dqw = geo_dquat.w;

		relative_pos_trans.vx = relative_pos_trans.dx / dt;
		relative_pos_trans.vy = relative_pos_trans.dy / dt;
		relative_pos_trans.vz = relative_pos_trans.dz / dt;

		relative_pos_trans.vqx = droll / dt;
		relative_pos_trans.vqy = dpitch/ dt;
		relative_pos_trans.vqz = dyaw  / dt;

		while(true){

			try{
				listener.waitForTransform("map", "velodyne", odom_data_time, ros::Duration(1.0) );
				listener.lookupTransform("velodyne", _last_odom_data_time, "velodyne", odom_data_time, "map", init_odometry_slide);
				ROS_INFO("GET TRANSFORM VELO FRAME AND VELODYNE FRAME IN GET_RELATIVE_TRANS");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(1.0).sleep();
			}
		}

	}
	
	void LaserOdometry::send_init_slide_tf(){

		geometry_msgs::Pose pos_pose = laserodometry.pose.pose;
		tf::Transform pos_pose_tf; //transform between map and laserodometry

		pos_pose_tf.setRotation( tf::Quaternion(pos_pose.orientation.x, pos_pose.orientation.y, pos_pose.orientation.z, pos_pose.orientation.w ) );
		pos_pose_tf.setOrigin( tf::Vector3(pos_pose.position.x, pos_pose.position.y, pos_pose.position.z) );


		tf::Transform map_to_init_slide = pos_pose_tf * init_odometry_slide;

		br.sendTransform( tf::StampedTransform(map_to_init_slide, _last_odom_data_time, "map", "init_slide"));
	}
	

/*	
	void LaserOdometry::send_init_slide_tf(){

		geometry_msgs::Pose pos_pose = laserodometry.pose.pose;
		tf::Quaternion pos_pose_quat;

		quaternionMsgToTF( pos_pose.orientation, pos_pose_quat );

		double cur_roll, cur_pitch, cur_yaw;
		tf::Matrix3x3( pos_pose_quat ).getRPY(cur_roll, cur_pitch, cur_yaw );

		cur_roll = cur_roll + relative_pos_trans.droll;
		cur_pitch = cur_pitch + relative_pos_trans.dpitch;
		cur_yaw = cur_yaw + relative_pos_trans.dyaw;

		tf::Quaternion init_slide_quat = tf::createQuaternionFromRPY( cur_roll, cur_pitch, cur_yaw);

		//tf::Quaternion trans_quat( relative_pos_trans.dqx, relative_pos_trans.dqy , relative_pos_trans.dqz , relative_pos_trans.dqw );

		//tf::Quaternion init_slide_quat = trans_quat * pos_pose_quat;
		//init_slide_quat.normalize();

		geometry_msgs::PoseStamped init_slide_pose;

		init_slide_pose.header.frame_id = "map";
		init_slide_pose.header.stamp = _last_odom_data_time;

		quaternionTFToMsg( init_slide_quat, init_slide_pose.pose.orientation );

		init_slide_pose.pose.position.x = pos_pose.position.x + relative_pos_trans.dx;
		init_slide_pose.pose.position.y = pos_pose.position.y + relative_pos_trans.dy;
		init_slide_pose.pose.position.z = pos_pose.position.z + relative_pos_trans.dz;

		_pub_init_slide_pose.publish(init_slide_pose);

		//std::cout << "init_slide_pose" << std::endl;
		//std::cout << init_slide_pose << std::endl;
		

		geometry_msgs::TransformStamped init_slide_transform;

		init_slide_transform.header.stamp = _last_odom_data_time;// koushinaito, lookuptransform dekinai
		init_slide_transform.header.frame_id = "map";
		init_slide_transform.child_frame_id = "init_slide";

		init_slide_transform.transform.translation.x = init_slide_pose.pose.position.x;
		init_slide_transform.transform.translation.y = init_slide_pose.pose.position.y;
		init_slide_transform.transform.translation.z = init_slide_pose.pose.position.z;

		init_slide_transform.transform.rotation = init_slide_pose.pose.orientation;

		br.sendTransform( init_slide_transform );
	}
	*/

	Eigen::Matrix4f LaserOdometry::new_init_pc_slide(){

		//send tf data between map and init_slide
		send_init_slide_tf();

		tf::StampedTransform laserodometry_to_init_slide;

		while(true){

			try{
				listener.waitForTransform("laserodometry", "init_slide", _last_odom_data_time, ros::Duration(1.0) );
				listener.lookupTransform("laserodometry", "init_slide", _last_odom_data_time, laserodometry_to_init_slide);

				ROS_INFO("GET TRANSFORM LASERODOMETRY FRAME AND INIT_SLIDE FRAME IN INIT_PC_SLIDE");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(0.1).sleep();
			}
		}

		Eigen::Matrix4f init_slide_matrix;

		pcl_ros::transformAsMatrix( laserodometry_to_init_slide , init_slide_matrix );

		return init_slide_matrix;


	}

	Eigen::Matrix4f LaserOdometry::new_pcl_pc_slide(){

		pcl::PointCloud<pcl::PointXYZRGB> icp_featurecloud;
		pcl::PointCloud<pcl::PointXYZRGB> icp_featurecloud_last;

		icp_featurecloud = FeatureCloud_child;
		icp_featurecloud.header.frame_id = "init_slide";

		icp_featurecloud_last = _lastFeatureCloud_child;
		icp_featurecloud_last.header.frame_id = "init_slide";

		tf::StampedTransform init_slide_to_laserodometry;

		while(true){

			try{
				listener.waitForTransform("init_slide", "laserodometry", _last_odom_data_time, ros::Duration(1.0) );
				listener.lookupTransform("init_slide", "laserodometry", _last_odom_data_time, init_slide_to_laserodometry);

				ROS_INFO("GET TRANSFORM INIT_SLIDE FRAME AND LASERODOMETRY FRAME IN INIT_PC_SLIDE");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(0.1).sleep();
			}
		}

		//slide last feature cloud to get 4x4 matrix in init slide coordinate by icp program
		pcl_ros::transformPointCloud( icp_featurecloud_last , icp_featurecloud_last , init_slide_to_laserodometry );

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr FeatureCloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>( icp_featurecloud ) );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr _lastFeatureCloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>( icp_featurecloud_last ) );

		//Set the input source(current scan) and target(previous scan) pointcloud
		//icp.setInputCloud( FeatureCloud_Ptr); // only ::Ptr
		//std::cout << "set source and target frame" << std::endl;
		icp.setInputSource( FeatureCloud_Ptr );
		icp.setInputTarget( _lastFeatureCloud_Ptr );

		// Set the max correspondence distance to 15cm (e.g., correspondences with higher distances will be ignored)
		//std::cout << "set ICP parameter" << std::endl;
		icp.setMaxCorrespondenceDistance(MaxCorrespondDistance);

		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);

		icp.align( tmp_pc_stored );

		//std::cout << "Do ICP" << std::endl;
		Eigen::Matrix4f pcl_slide_matrix = icp.getFinalTransformation();

		//std::cout <<"PCL Matrix" << std::endl;
		//std::cout << pcl_slide_matrix << std::endl;

		return pcl_slide_matrix;
	}



	Eigen::Matrix4f LaserOdometry::pcl_pc_slide(){

		//std::cout << "Start ICP" << std::endl;

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr FeatureCloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>( FeatureCloud ) );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr _lastFeatureCloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>( _lastFeatureCloud ) );


		//Set the input source(current scan) and target(previous scan) pointcloud
		//icp.setInputCloud( FeatureCloud_Ptr); // only ::Ptr
		//std::cout << "set source and target frame" << std::endl;
		icp.setInputSource( FeatureCloud_Ptr );
		icp.setInputTarget( _lastFeatureCloud_Ptr );

		// Set the max correspondence distance to 15cm (e.g., correspondences with higher distances will be ignored)
		//std::cout << "set ICP parameter" << std::endl;
		icp.setMaxCorrespondenceDistance(MaxCorrespondDistance);

		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);

		icp.align( tmp_pc_stored );

		//std::cout << "Do ICP" << std::endl;
		Eigen::Matrix4f pcl_slide_matrix = icp.getFinalTransformation();

		//std::cout <<"PCL Matrix" << std::endl;
		//std::cout << pcl_slide_matrix << std::endl;
		
		pcl::transformPointCloud( FeatureCloud , FeatureCloud , pcl_slide_matrix );


		return pcl_slide_matrix;
	}



	void LaserOdometry::get_tf_data(){

		while(true){

			try{
				listener.lookupTransform("map", "velodyne", _last_odom_data_time, velo_to_map);
				listener.lookupTransform("map", "laserodometry", _last_odom_data_time, laserodometry_to_map);
				//listener.lookupTransform("map", "velodyne", _last_odom_data_time, listener);
				//listener.lookupTransform("map", "laserodometry", _last_odom_data_time, listener);


				ROS_INFO("GET TRANSFORM MAP FRAME AND VELODYNE FRAME IN GET_TF_DATA");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(1.0).sleep();
			}
		}

		//std::cout << "get transform" << std::endl;

	}

	
	void LaserOdometry::convert_coordinate_of_pc(){

		//std::cout << "Transform coordinate" << std::endl;
		
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, FeatureCloud_child, "laserodometry", FeatureCloud, listener);
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, velo_scans_child, "laserodometry", velo_scans, listener);

		//std::cout << FeatureCloud.header.frame_id << std::endl;
		//std::cout << "convert coordinate" << std::endl;

	}

	bool LaserOdometry::initialization(){
		//CloudCentroid.swap(_lastCloudCentroid);
		//CloudEdge.swap(_lastCloudEdge);
		//FeatureCloud.swap( _lastFeatureCloud );

		_last_CloudCentroid_time = CloudCentroid_time;
		_last_CloudEdge_time = CloudEdge_time;

		_last_odom_data = odom_data;
		_last_odom_data_time = odom_data_time;

		// getting laser odometry's init parameter
		laserodometry.pose.pose.position = _last_odom_data.pose.pose.position;
		laserodometry.pose.pose.orientation = _last_odom_data.pose.pose.orientation;
		laserodometry.twist.twist.linear = _last_odom_data.twist.twist.linear;
		laserodometry.twist.twist.angular = _last_odom_data.twist.twist.angular;
		
		laserodometry_to_map.stamp_ = _last_odom_data_time;
		laserodometry_to_map.setRotation( tf::Quaternion( laserodometry.pose.pose.orientation.x, laserodometry.pose.pose.orientation.y, laserodometry.pose.pose.orientation.z, laserodometry.pose.pose.orientation.w ) );
		laserodometry_to_map.setOrigin( tf::Vector3( laserodometry.pose.pose.position.x, laserodometry.pose.pose.position.y, laserodometry.pose.pose.position.z) );

		br.sendTransform( laserodometry_to_map );

		//std::cout << "send init tf" << std::endl;
		

		while(true){
			try{
				listener.waitForTransform("map", "laserodometry", _last_odom_data_time, ros::Duration(1.0) );
				listener.lookupTransform("map", "laserodometry", _last_odom_data_time, laserodometry_to_map);
				//listener.lookupTransform("map", "laserodometry", _last_odom_data_time, listener);
				ROS_INFO("GET TRANSFORM MAP FRAME AND VELODYNE FRAME IN INITIALIZATION");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(1.0).sleep();
			}
		}

		//Transform previous scans
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, _lastCloudCentroid, "laserodometry", _lastCloudCentroid, listener);
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, _lastCloudEdge, "laserodometry", _lastCloudEdge, listener);
		
		//std::cout << "pcl trans" << std::endl;
		pcl_ros::transformPointCloud("map", _last_odom_data_time, FeatureCloud_child, "laserodometry", FeatureCloud, listener );

		_lastFeatureCloud = FeatureCloud;
		_lastFeatureCloud_child = FeatureCloud_child;
		FeatureCloud.clear();
		return true;

	}

	void LaserOdometry::send_tf_data(Eigen::Matrix4f Tm){

		tf::Matrix3x3 rota;
		tf::Quaternion rot_quat;

		quaternionMsgToTF( laserodometry.pose.pose.orientation, rot_quat );

		rota.setRotation( rot_quat );

		tf::Vector3 place;
		place.setValue(  laserodometry.pose.pose.position.x, laserodometry.pose.pose.position.y, laserodometry.pose.pose.position.z );

		Eigen::Matrix4f last_pose;

		/*
		tf::Vector3 row0 = rota.getRow(0);
		tf::Vector3 row1 = rota.getRow(1);
		tf::Vector3 row2 = rota.getRow(2);

		
		last_pose <<               row0.getX(), row0.getY(), row0.getZ(), place.x(),
				           row1.getX(), row1.getY(), row1.getZ(), place.y(),
				           row2.getX(), row2.getY(), row2.getZ(), place.z(),
				                  0.0,        0.0,        0.0,     1.0;

		

		*/
		last_pose <<               rota[0][0], rota[0][1], rota[0][2], place[0],
				           rota[1][0], rota[1][1], rota[1][2], place[1],
				           rota[2][0], rota[2][1], rota[2][2], place[2],
				                  0.0,        0.0,        0.0,     1.0;
		//std::cout << "last_pose" << std::endl;
		//std::cout << last_pose << std::endl;
	//std::cout << "getting now pose as 4x4 matrix" << std::endl;
		Eigen::Matrix4f now_pose = last_pose * Tm;

		/*
		tf::Matrix3x3 orientation_mat(
				now_pose(0,0), now_pose(0,1), now_pose(0,2),
				now_pose(1,0), now_pose(1,1), now_pose(1,2),
				now_pose(2,0), now_pose(2,1), now_pose(2,2) );

		tf::Quaternion quat;

		orientation_mat.getRotation( quat );
		quat.normalize();

		geometry_msgs::Pose cur_pose;
		
		quaternionTFToMsg( quat , cur_pose.orientation );
		*/

		Eigen::Matrix3f ori_mat;
		ori_mat << now_pose(0, 0), now_pose(0, 1), now_pose(0, 2),
			   now_pose(1, 0), now_pose(1, 1), now_pose(1, 2),
			   now_pose(2, 0), now_pose(2, 1), now_pose(2, 2);

		//std::cout << "now_pose" << std::endl;
		//std::cout << now_pose << std::endl;

		//std::cout << "ori_mat" << std::endl;
		//std::cout << ori_mat << std::endl;

		double angle_x, angle_y, angle_z;
		
		double PI = 3.141592;
		double threshold = 0.001;

		if( abs(ori_mat(2, 1) - 1.0 ) < threshold ){
			angle_x = PI/2;
			angle_y = 0;
			angle_z = atan2( ori_mat(1, 0), ori_mat(0, 0) );
		}
		else if( abs( ori_mat(2, 1) + 1.0 ) < threshold ){
			angle_x = - PI/2;
			angle_y = 0;
			angle_z = atan2( ori_mat(1, 0), ori_mat(0, 0) );
		}
		else{
			angle_x = asin( ori_mat(2, 1) );
			angle_y = atan2( -ori_mat(2, 0), ori_mat(2, 2) );
			angle_z = atan2( -ori_mat(0, 1), ori_mat(1, 1) );
		}

		tf::Quaternion now_quat = tf::createQuaternionFromRPY( angle_x, angle_y, angle_z );
		geometry_msgs::Quaternion geo_quat;
		quaternionTFToMsg( now_quat, geo_quat );

		geometry_msgs::Pose cur_pose;
		cur_pose.orientation = geo_quat;

		//dainyuu
		cur_pose.position.x = now_pose(0,3);
		cur_pose.position.y = now_pose(1,3);
		cur_pose.position.z = now_pose(2,3);

		tf::Transform calib_transform;
		poseMsgToTF( cur_pose , calib_transform );

		//std::cout << cur_pose << std::endl;

		//std::cout << "send current TF data" << std::endl;

		//br.sendTransform( tf::StampedTransform( calib_transform, odom_data_time, "map", "laserodometry") );

		//rewrite laserodometry
		laserodometry.pose.pose = cur_pose;


		laserodometry.twist.twist.linear.x = relative_pos_trans.vx;
		laserodometry.twist.twist.linear.y = relative_pos_trans.vy;
		laserodometry.twist.twist.linear.z = relative_pos_trans.vz;
		
		laserodometry.twist.twist.angular.x = relative_pos_trans.vqx;
		laserodometry.twist.twist.angular.y = relative_pos_trans.vqy;
		laserodometry.twist.twist.angular.z = relative_pos_trans.vqz;
		

		laserodometry.header.stamp = odom_data_time;

		geometry_msgs::TransformStamped car_state;

		car_state.header.stamp = laserodometry.header.stamp;
		car_state.header.frame_id = "map";
		car_state.child_frame_id = "laserodometry";

		car_state.transform.translation.x = laserodometry.pose.pose.position.x;
		car_state.transform.translation.y = laserodometry.pose.pose.position.y;
		car_state.transform.translation.z = laserodometry.pose.pose.position.z;


		car_state.transform.rotation = laserodometry.pose.pose.orientation;


		br.sendTransform( car_state );

		/*
		// rewrite laserodometry_to_map
		laserodometry_to_map.stamp_ = laserodometry.header.stamp;
		laserodometry_to_map.setRotation( tf::Quaternion( laserodometry.pose.pose.orientation.x, laserodometry.pose.pose.orientation.y, laserodometry.pose.pose.orientation.z, laserodometry.pose.pose.orientation.w ) );
		laserodometry_to_map.setOrigin( tf::Vector3( laserodometry.pose.pose.position.x, laserodometry.pose.pose.position.y, laserodometry.pose.pose.position.z) );

		br.sendTransform( laserodometry_to_map );
		*/
	}

	void LaserOdometry::final_transform_pc(Eigen::Matrix4f laserodometry_trans_matrix){

		pcl::transformPointCloud( velo_scans , velo_scans , laserodometry_trans_matrix );
		//pcl::transformPointCloud( FeatureCloud , FeatureCloud , laserodometry_trans_matrix );

	}

	void LaserOdometry::publish_result(){

		sensor_msgs::PointCloud2 velo, cent, edge;
		//sensor_msgs::PointCloud2 velo, feature;

		pcl::toROSMsg(velo_scans_child, velo);
		velo.header.frame_id = "laserodometry";
		velo.header.stamp = odom_data_time;

		//pcl::toROSMsg(FeatureCloud, feature);
		//feature.header.frame_id = 
		
		pcl::toROSMsg(CloudCentroid, cent);
		cent.header.frame_id = "laserodometry";
		cent.header.stamp = odom_data_time;

		pcl::toROSMsg(CloudEdge, edge);
		edge.header.frame_id = "laserodometry";
		edge.header.stamp = odom_data_time;
		

		_pubVelodynePoints3.publish(velo);
		_pubCentroidPointLast.publish(cent);
		_pubEdgePointLast.publish(edge);
		_pubLaserOdomToInit.publish(laserodometry);

		/*
		std::cout << "laserodometry" << std::endl;
		std::cout << laserodometry << std::endl;

		std::cout << "odom_data" << std::endl;
		std::cout << odom_data << std::endl;
		*/

	}

	void LaserOdometry::reset(){
		tmp_pc_stored.clear();

		velo_scans.clear();
		velo_scans_child.clear();

		CloudCentroid.clear();
		CloudEdge.clear();

		_lastFeatureCloud = FeatureCloud;
		_lastFeatureCloud_child = FeatureCloud_child;

		FeatureCloud.clear();
		FeatureCloud_child.clear();

		_last_odom_data = odom_data;

		_last_odom_data_time = odom_data_time;

		velo_scans_checker = false;
		CloudCentroid_checker = false;
		CloudEdge_checker = false;
		odom_data_checker = false;

	}

	void LaserOdometry::process(){

		if( __systemdelay > 0 ){

			--__systemdelay;
			return;
		}

		if(!system_initialized_checker){
			bool init_bool = initialization();
			if(init_bool){
				system_initialized_checker = true;
				return;
			}
		}

		bool status = hasNewData();

		if( status == false )
			return;

		get_tf_data();

		get_relative_trans();

		// Convert point cloud's coordinate velodyne frame to map frame
		convert_coordinate_of_pc();

		//Calculate init slide as homogenerous transformation matrix by odometry
		//Eigen::Matrix4f init_slide_matrix = init_pc_slide();
		Eigen::Matrix4f init_slide_matrix = new_init_pc_slide();

		//Calculate ICP slide by pcl's Iterative Closest Point module
		//Eigen::Matrix4f pcl_slide_matrix = pcl_pc_slide();
		Eigen::Matrix4f pcl_slide_matrix = new_pcl_pc_slide();

		//Calculate calibrated laserodometry as homogenerous transformation matrix
		Eigen::Matrix4f laserodometry_trans_matrix = init_slide_matrix * pcl_slide_matrix;
		//Eigen::Matrix4f laserodometry_trans_matrix = init_slide_matrix;

		// Calculate calibrated odometry data and publish tf data
		send_tf_data(laserodometry_trans_matrix);

		// transform to calibrated position
		final_transform_pc(laserodometry_trans_matrix);

		publish_result();

		reset();

	}

	void LaserOdometry::spin(){

		ros::Rate rate(10);

		while( ros::ok() ){

			ros::spinOnce(); //get published data

			process();

			rate.sleep();
		}

	}

}


