#include"semantic_mesh_loam/laser_mapping.h"

namespace semloam{

	LaserMapping::LaserMapping(){

		velo_scans.reserve(velo_scan_size);
		cloud_centroid.reserve(feature_scan_size);
		cloud_edge.reserve(feature_scan_size);

		unlabeled.reserve(pc_size_min);
		outlier.reserve(pc_size_min);
		car.reserve(pc_size_mid);
		bicycle.reserve(pc_size_min);
		motorcycle.reserve(pc_size_min);
		onrails.reserve(pc_size_min);
		truck.reserve(pc_size_mid);
		othervehicle.reserve(pc_size_mid);
		person.reserve(pc_size_min);
		bicyclist.reserve(pc_size_min);
		motorcyclist.reserve(pc_size_min);
		road.reserve(pc_size_big);
		parking.reserve(pc_size_mid);
		sidewalk.reserve(pc_size_mid);
		otherground.reserve(pc_size_mid);
		building.reserve(pc_size_big);
		fence.reserve(pc_size_big);
		otherstructure.reserve(pc_size_min);
		lanemarking.reserve(pc_size_min);
		vegetation.reserve(pc_size_mid);
		trunk.reserve(pc_size_min);
		terrain.reserve(pc_size_mid);;
		pole.reserve(pc_size_min);
		trafficsign.reserve(pc_size_min);



	}

	void LaserMapping::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata){

		pcl::fromROSMsg( *clouddata , velo_scans );

		velo_checker = true;

		std::cout << "catch velodyne_data" << std::endl;

	}

	void LaserMapping::edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata){

		pcl::fromROSMsg( *edgedata , cloud_edge );

		edge_checker = true;

		std::cout << "catch edge_data" << std::endl;

	}

	void LaserMapping::centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata){

		pcl::fromROSMsg( *centroiddata , cloud_centroid );

		cent_checker = true;

		std::cout << "catch cent_data" << std::endl;

	}

	void LaserMapping::odometry_callback(const nav_msgs::OdometryConstPtr& odomdata){

		odom_data = *odomdata;

		odom_time = odom_data.header.stamp;

		odom_checker = true;

		std::cout << "catch odom_data" << std::endl;

	}


	bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

		_sub_odometry = node.subscribe<nav_msgs::Odometry>
			("/laser_odom_to_init", 10, &LaserMapping::odometry_callback, this);
		_sub_velodyne = node.subscribe<sensor_msgs::PointCloud2>
			("/velodyne_points3", 2, &LaserMapping::velodyne_callback, this);
		_sub_edge = node.subscribe<sensor_msgs::PointCloud2>
			("/edge_point_last", 2, &LaserMapping::edge_callback, this);
		_sub_centroid = node.subscribe<sensor_msgs::PointCloud2>
			("/centroid_point_last", 2, &LaserMapping::centroid_callback, this);


		std::cout << "Set ros::Subscribe parameter" << std::endl;

		float fparam;
		int iparam;

		if( privateNode.getParam("scan_counter", iparam) ){
			if( iparam < 1 ){
				ROS_ERROR("Invalid scan_counter parameter: %d", iparam);
				return false;
			}
			else{
				scan_counter = iparam;
			}
		}

		// Parameter from launch file


		return true;
	}

	void LaserMapping::get_tf_data(){
		//Look up transform between map and laserodometry
		while(true){
			try{
				listener.waitForTransform("map", "laserodometry", odom_time, ros::Duration(0.5) );
				listener.lookupTransform("map", "laserodometry", odom_time, map_to_laserodometry );

				ROS_INFO("GET TRANSFORM MAP TO KASERODOMETRY IN LASERMAPPING PROCESS");
				
				break;
			}
			catch(tf::TransformException ex){
				ROS_ERROR("TF ERROR AT LASERMAPPING %s", ex.what() );
				ros::Duration(0.1).sleep();
			}
		}
	}

	bool LaserMapping::classify(const pcl::PointXYZRGB& point, const color_data& color_id){
		if(color_id.r==0 && color_id.g==0 && color_id.b==0){
			unlabeled.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
			outlier.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==150 && color_id.b==245){
			car.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==230 && color_id.b==245){
			bicycle.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==80 && color_id.b==250){
			bus.push_back(point);
		}
		else if(color_id.r==30 && color_id.g==60 && color_id.b==150){
			motorcycle.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==0 && color_id.b==255){
			onrails.push_back(point);
		}
		else if(color_id.r==80 && color_id.g==30 && color_id.b==180){
			truck.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==30 && color_id.b==30){
			person.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==40 && color_id.b==200){
			bicyclist.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==30 && color_id.b==90){
			motorcyclist.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==255){
			road.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==255){
			parking.push_back(point);
		}
		else if(color_id.r==75 && color_id.g==0 && color_id.b==75){
			sidewalk.push_back(point);
		}
		else if(color_id.r==175 && color_id.g==0 && color_id.b==75){
			otherground.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==200 && color_id.b==0){
			building.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==120 && color_id.b==50){
			fence.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==0){
			otherstructure.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==255 && color_id.b==170){
			lanemarking.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==175 && color_id.g==0){
			vegetation.push_back(point);
		}
		else if(color_id.r==135 && color_id.g==60 && color_id.b==0){
			trunk.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==240 && color_id.b==80){
			terrain.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==240 && color_id.b==150){
			pole.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
			trafficsign.push_back(point);
		}

		return true;
	}

	void LaserMapping::classify_pointcloud(){

		size_t cloud_size = velo_scans.size();

		pcl::PointXYZRGB point;

		for(int i=0; i < cloud_size; i++){

			point.x = velo_scans[i].x;
			point.y = velo_scans[i].y;
			point.z = velo_scans[i].z;

			point.r = velo_scans[i].r;
			point.g = velo_scans[i].g;
			point.b = velo_scans[i].b;

			color_data color_id;
			color_id.r = int(point.r);
			color_id.g = int(point.g);
			color_id.b = int(point.b);


			if(             !pcl_isfinite(point.x) ||
					!pcl_isfinite(point.y) ||
					!pcl_isfinite(point.z)){
				continue;
			}

			if(point.x*point.x + point.y*point.y + point.z*point.z < 0.00001){
				continue;
			}

			//classify point and contain to each each semantic points
			if(!classify(point, color_id)){
				//continue;
			}

		}

		std::cout << "End classify pointcloud" << std::endl;

	}

	void LaserMapping::convert_coordinate_of_pc(){

		pcl_ros::transformPointCloud("map", odom_time, velo_scans, "laserodometry", velo_scans, listener);
		//pcl_ros::transformPointCloud("map", odom_time, cloud_edge, "laserodometry", cloud_edge, listener);
		//pcl_ros::transformPointCloud("map", odom_time, cloud_centroid, "laserodometry", cloud_centroid, listener);

		std::cout << "PointCloud is transformed from laserodometry frame to map frame" << std::endl;
	}

	void LaserMapping::reset_common_cloud(){

		velo_scans.clear();
		cloud_edge.clear();
		cloud_centroid.clear();

		std::cout << "clear common cloud" << std::endl;
	}

	void LaserMapping::reset_semantic_cloud(){

		//clear semantic point cloud data
		unlabeled.clear();
		outlier.clear();
		car.clear();
		bicycle.clear();
		motorcycle.clear();
		onrails.clear();
		truck.clear();
		othervehicle.clear();
		person.clear();
		bicyclist.clear();
		motorcyclist.clear();
		road.clear();
		parking.clear();
		sidewalk.clear();
		otherground.clear();
		building.clear();
		fence.clear();
		otherstructure.clear();
		lanemarking.clear();
		vegetation.clear();
		trunk.clear();
		terrain.clear();
		pole.clear();
		trafficsign.clear();

		std::cout << "clear semantic cloud" << std::endl;
	}

	void LaserMapping::process(int counter){

		Time start_sec = ros::Time::now();

		//get transform between map to laserodometry
		get_tf_data();

		//convert PointCloud's coordinate from laserodometry to map frame
		convert_coordinate_of_pc();

		//classify pointcloud
		classify_pointcloud();

		if( (counter % scan_counter) == 0 ){
			//do voxel grid reducion per semantic point cloud
			do_voxel_grid();

			//publish pointcloud as ROS msg
			publish_pointcloud();

			//publish pointcloud as .PCD file
			//pointcloud_to_pcd();
			
			//clear semantic point cloud like car and outlier
			reset_semantic_cloud();
		}

		//clear common pointcloud like
		reset_common_cloud();

		Time end_sec = ros::Time::now();

		double duration_process = end_sec.toSec() - start_sec.toSec();

		std::cout << "process has ended" << std::endl;
		std::cout << "process time: "<< duration_process << std::endl;

	}



	void LaserMapping::spin(){

		ros::Rate rate(10);
		
		int counter = 0;

		while( ros::ok() ){

			counter += 1;

			ros::sinOnce();

			process(counter);

			rate.sleep();
		}
	}

}

