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
		_subEdge = node.subscriber<sensor_msgs::PointCloud2>
			("/edge_points", 2, &LaserOdometry::edge_callback, this);
		_subOdometry = node.subscribe<sensor_msgs::PointCloud2>
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

		std::cout << "catch centroid data" << std::endl;

		CloudCentroid_checker = true;
	}

	void LaserOdometry::edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata){
		CloudEdge_time = edgedata->header.stamp;

		pcl::fromROSMsg(*edgedata, CloudEdge);

		std::cout << "catch edge data" << std::endl;

		CloudEdge_checker = true;
	}

	void LaserOdometry::odometry_callback(const nav_msgs::OdometryConstPtr& odomdata){
		odom_data_time = odomdata->header.stamp;

		odom_data = *odomdata;

		std::cout << "catch odom data" << std::endl;

		odom_data_checker = true;
	}

	void LaserOdometry::spin(){

		ros::Rate rate(100);

		while( ros::ok() ){

			ros::spinOnce(); //get published data

			if(scancount == 0){
				get_init_data();
			}
			else{
				get_cur_data();

				process();
			}

			if(scancount + 1 > 10000000){
				scancount = scancount;
			}
			else{
				scancount += 1;
			}

			rate.sleep();
		}

	}

}


