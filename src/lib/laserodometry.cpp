#include"semantic_mesh_loam/laser_odometry.h"


namespace semloam{

	LaserOdometry::LaserOdometry(void){

		scanperiod = 0.1;
		ioratio = 2;
		max_iterations = 25;
		delta_t_abort = 0.1;
		delta_r_abort = 0.1;

		
		velo_scans.reserve(scan_size);
		CloudCentroid.reserve(feature_size);
		CloudEdge.reserve(feature_size);
		FeatureCloud.reserve(feature_size);
		
		_lastCloudCentroid.reserve(feature_size);
		_lastCloudEdge.reserve(feature_size);
		_lastFeatureCloud.reserve(feature_size);

		CloudCentroidInd.reserve(ind_size);
		_lastCloudCentroidInd.reserve(ind_size);

		CloudEdgeInd.reserve(ind_size);
		_lastCloudEdgeInd.reserve(ind_size);

		odom_data.header.frame_id = "map";
		odom_data.child_frame_id = "vehicle";

		_last_odom_data.header.frame_id = "map";
		_last_odom_data.child_frame_id = "vehicle";

		laserodometry.header.frame_id = "map";
		laserodometry.child_frame_id = "laserodometry";

		/*
		odometrytrans.header.frame_id = "map";
		odometrytrans.child_frame_id = "vehicle";
		*/

		system_initialized_checker = false;

	}

	bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode){

		float fparam;
		int iparam;

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

		_pubLaserOdomToInit = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
		_pubVelodynePoints3 = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points3", 2);
		_pubCentroidPointLast = node.advertise<sensor_msgs::PointCloud2>("/centroid_point_last", 2);
		_pubEdgePointLast = node.advertise<sensor_msgs::PointCloud2>("/edge_point_last", 2);


		//subscribe
		_subVelodynePoints = node.subscribe<sensor_msgs::PointCloud2>
			("/velodyne_points2", 2, &LaserOdometry::velodyne_callback, this);
		_subCentroid = node.subscribe<sensor_msgs::PointCloud2>
			("/centroid_points", 2, &LaserOdometry::centroid_callback, this);
		_subEdge = node.subscribe<sensor_msgs::PointCloud2>
			("/edge_points", 2, &LaserOdometry::edge_callback, this);
		_subOdometry = node.subscribe<nav_msgs::Odometry>
			("/odometry", 2, &LaserOdometry::odometry_callback, this);


		return true;
	}

	void LaserOdometry::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata){
		velo_scans_time = clouddata->header.stamp;
		
		pcl::fromROSMsg(*clouddata, velo_scans);

		std::cout << "velo data catched" << std::endl;

		velo_scans_checker = true;
	}

	void LaserOdometry::centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata){
		CloudCentroid_time = centroiddata->header.stamp;

		pcl::fromROSMsg(*centroiddata, CloudCentroid);

		FeatureCloud = FeatureCloud + CloudCentroid;

		std::cout << "catch centroid data" << std::endl;

		CloudCentroid_checker = true;
	}

	void LaserOdometry::edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata){
		CloudEdge_time = edgedata->header.stamp;

		pcl::fromROSMsg(*edgedata, CloudEdge);

		FeatureCloud = FeatureCloud + CloudEdge;

		std::cout << "catch edge data" << std::endl;

		CloudEdge_checker = true;
	}

	void LaserOdometry::odometry_callback(const nav_msgs::OdometryConstPtr& odomdata){
		
		_last_odom_data_time = odom_data_time;

		odom_data_time = odomdata->header.stamp;

		odom_data = *odomdata;

		std::cout << "catch odom data" << std::endl;

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

	void LaserOdometry::get_relative_trans(){

		relative_pos_trans.dx = odom_data.pose.pose.position.x - _last_odom_data.pose.pose.position.x;
		relative_pos_trans.dy = odom_data.pose.pose.position.y - _last_odom_data.pose.pose.position.y;
		relative_pos_trans.dz = odom_data.pose.pose.position.z - _last_odom_data.pose.pose.position.z;

		relative_pos_trans.dqx = odom_data.pose.pose.orientation.x - _last_odom_data.pose.pose.orientation.x;
		relative_pos_trans.dqy = odom_data.pose.pose.orientation.y - _last_odom_data.pose.pose.orientation.y;
		relative_pos_trans.dqz = odom_data.pose.pose.orientation.z - _last_odom_data.pose.pose.orientation.z;
		relative_pos_trans.dqw = odom_data.pose.pose.orientation.w - _last_odom_data.pose.pose.orientation.w;

	}

	/*
	void LaserOdometry::init_pc_slide(){
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;

		quaternionMsgToTF(odom_data.pose.pose.orientation, pose_now);
		quaternionMsgToTF( _last_odom_data.pose.pose.orientation, pose_last);

		tf::Quaternion relative_rotation = pose_last * pose_now.inverse();
		relative_rotation.normalize();

		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z() );

		tf::Quaternion q_global_move(

				_last_odom_data.pose.pose.position.x - odom_data.pose.pose.position.x,
				_last_odom_data.pose.pose.position.y - odom_data.pose.pose.position.y,
				_last_odom_data.pose.pose.position.z - odom_data.pose.pose.position.z,
				0.0 );
		
		tf::Quaternion q_local_move = pose_last.inverse() * q_global_move * pose_last;
		Eigen::Vector3f offset( q_local_move.x(), q_local_move.y(), q_local_move.z() );

		// transform previous scans
		pcl::transformPointCloud( _lastCloudCentroid, _lastCloudCentroid, offset, rotation );
		pcl::transformPointCloud( _lastCloudEdge, _lastCloudEdge, offset, rotation );

	}*/

	void LaserOdometry::init_pc_slide(){
		//a
	}

	void LaserOdometry::get_tf_data(){

		while(true){

			try{
				listener.lookupTransform("map", "velodyne", _last_odom_data_time, velo_to_map);
				listener.lookupTransform("map", "laserodometry", _last_odom_data_time, laserodometry_to_map);
				ROS_INFO("GET TRANSFORM MAP FRAME AND VELODYNE FRAME");
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what() );
				ros::Duration(1.0).sleep();
			}
		}

		std::cout << "get transform" << std::endl;

	}

	
	void LaserOdometry::convert_coordinate_of_pc(){

		//Transforming current scans
		pcl_ros::transformPointCloud("map", _last_odom_data_time, velo_scans, "laserodometry", velo_scans, listener);
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, CloudCentroid, "laserodometry", CloudCentroid, listener);
		//pcl_ros::transformPointCloud("map", _last_odom_data_time, CloudEdge, "laserodometry", CloudEdge, listener);

		pcl_ros::transformPointCloud("map", _last_odom_data_time, FeatureCloud, "laserodometry", FeatureCloud, listener);

	}

	bool LaserOdometry::initialization(){
		//CloudCentroid.swap(_lastCloudCentroid);
		//CloudEdge.swap(_lastCloudEdge);
		FeatureCloud.swap( _lastFeatureCloud );

		_last_CloudCentroid_time = CloudCentroid_time;
		_last_CloudEdge_time = CloudEdge_time;

		_last_odom_data = odom_data;
		_last_odom_data_time = odom_data_time;

		// getting laser odometry's init parameter
		laserodometry.pose.pose.position = _last_odom_data.pose.pose.position;
		laserodometry.pose.pose.orientation = _last_odom_data.pose.pose.orientation;
		laserodometry.twist.twist.linear = _last_odom_data.twist.twist.linear;
		laserodometry.twist.twist.angular = _last_odom_data.twist.twist.angular;

		geometry_msgs::Pose init_pose;
		init_pose.position = laserodometry.pose.pose.position;
		init_pose.orientation = laserodometry.pose.pose.orientation;
		
		tf::Transform init_transform;
		poseMsgToTF(init_pose, init_transform);
		br.sendTransform(tf::StampedTransform(init_transform, _last_odom_data_time, "map", "laserodometry"));

		while(true){
			try{
				listener.lookupTransform("map", "laserodometry", _last_odom_data_time, laserodometry_to_map);
				ROS_INFO("GET TRANSFORM MAP FRAME AND VELODYNE FRAME");
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
		pcl_ros::transformPointCloud("map", _last_odom_data_time, _lastFeatureCloud, "laserodometry", _lastFeatureCloud, listener );

		return true;

	}

	void LaserOdometry::process(){

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

		get_relative_trans();

		get_tf_data();

		// Convert point cloud's coordinate velodyne frame to map frame
		convert_coordinate_of_pc();

		// slide point cloud position
		init_pc_slide();

	}

	void LaserOdometry::spin(){

		ros::Rate rate(100);

		rate.sleep();

		while( ros::ok() ){

			ros::spinOnce(); //get published data

			process();

			rate.sleep();
		}

	}

}


