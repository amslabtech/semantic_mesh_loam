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
		//person.reserve(pc_size_min);
		//bicyclist.reserve(pc_size_min);
		//motorcyclist.reserve(pc_size_min);
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

		pc_map_cloud.reserve( pc_map_cloud_size );

	}

	void LaserMapping::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata){

		pcl::fromROSMsg( *clouddata , velo_scans );
        velo_scans.header.frame_id = reference_tf_frame_name;

		velo_checker = true;

		//std::cout << "catch velodyne_data" << std::endl;

	}

	void LaserMapping::edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata){

		pcl::fromROSMsg( *edgedata , cloud_edge );

		edge_checker = true;

		//std::cout << "catch edge_data" << std::endl;

	}

	void LaserMapping::centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata){

		pcl::fromROSMsg( *centroiddata , cloud_centroid );

		cent_checker = true;

		//std::cout << "catch cent_data" << std::endl;

	}

	void LaserMapping::odometry_callback(const nav_msgs::OdometryConstPtr& odomdata){

		odom_data = *odomdata;

		odom_time = odom_data.header.stamp;

		odom_checker = true;

		//std::cout << "catch odom_data" << std::endl;

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

		_pub_pc = node.advertise<sensor_msgs::PointCloud2>("/final_pointcloud",1);
		_pub_odom = node.advertise<nav_msgs::Odometry>("/final_odometry",10);

		std::cout << "Set ros::Subscribe parameter" << std::endl;

		float fparam;
		int iparam;
		bool bparam;
		std::string strparam;

        if( privateNode.getParam("ReferenceFrame" , strparam ) ){
            if( strparam=="laserodometry" || strparam=="ground_truth" || strparam=="vehicle"){
                reference_tf_frame_name = strparam;
                std::cout << "Reference frame: " << reference_tf_frame_name << std::endl;
            }
            else{
                ROS_ERROR("Invalid TF frame name");
                return false;
            }
        }

		if( privateNode.getParam("filepath", strparam)){
			if( strparam.length() < 1 ){
				ROS_ERROR("Invalid file path");
				return false;
			}
			else{
				file_path = strparam;
			}
		}

		if( privateNode.getParam("filename", strparam)){
			if( strparam.length() < 1 ){
				ROS_ERROR("Invalid file name");
				return false;
			}
			else{
				file_name = strparam;
			}
		}

		if( privateNode.getParam("PCDsaveAscii", bparam)){
			if( !(bparam==true || bparam==false) ){
				ROS_ERROR("Invalid PCDsaveAscii parameter");
				return false;
			}
			else{
				pcd_save_ascii = bparam;
			}
		}

		if( privateNode.getParam("PCDsaveBinary", bparam)){
			if( !(bparam==true || bparam==false) ){
				ROS_ERROR("Invalid PCDsaveBinary parameter");
				return false;
			}
			else{
				pcd_save_binary = bparam;
			}
		}

		if( privateNode.getParam("PCDsaveChecker", bparam)){
			if( !(bparam==true || bparam==false) ){
				ROS_ERROR("Invalid PCDsaveChecker parameter");
				return false;
			}
			else{
				pcd_save_checker = bparam;
			}
		}

		if( privateNode.getParam("ROSpublishChecker", bparam)){
			if( !(bparam==true || bparam==false) ){
				ROS_ERROR("Invalid ROSPublishChecker parameter");
				return false;
			}
			else{
				ros_publish_checker = bparam;
			}
		}

		if( privateNode.getParam("scancounter", iparam) ){
			if( iparam < 1 ){
				ROS_ERROR("Invalid scan_counter parameter: %d", iparam);
				return false;
			}
			else{
				scan_counter = iparam;
			}
		}

		if( privateNode.getParam("systemdelay", iparam) ){
			if( iparam < 1 ){
				ROS_ERROR("Invalid systemdelay parameter: %d", iparam);
				return false;
			}
			else{
				systemdelay = iparam;
			}
		}

		if( privateNode.getParam("unlabeledleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("unlabeled_leafsize parameter error");
				return false;
			}
			else{
				unlabeled_leafsize = fparam;
			}
		}

		if( privateNode.getParam("outlierleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("outlier_leafsize parameter error");
				return false;
			}
			else{
				outlier_leafsize = fparam;
			}
		}

		if( privateNode.getParam("carleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("car_leafsize parameter error");
				return false;
			}
			else{
				car_leafsize = fparam;
			}
		}

		if( privateNode.getParam("bicycleleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("bicycle_leafsize parameter error");
				return false;
			}
			else{
				bicycle_leafsize = fparam;
			}
		}

		if( privateNode.getParam("busleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("bus_leafsize parameter error");
				return false;
			}
			else{
				bus_leafsize = fparam;
			}
		}

		if( privateNode.getParam("motorcycleleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("motorcycle_leafsize parameter error");
				return false;
			}
			else{
				motorcycle_leafsize = fparam;
			}
		}

		if( privateNode.getParam("onrailsleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("onrails_leafsize parameter error");
				return false;
			}
			else{
				onrails_leafsize = fparam;
			}
		}

		if( privateNode.getParam("truckleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("truck_leafsize parameter error");
				return false;
			}
			else{
				truck_leafsize = fparam;
			}
		}

		if( privateNode.getParam("othervehicleleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("othervehicle_leafsize parameter error");
				return false;
			}
			else{
				othervehicle_leafsize = fparam;
			}
		}

		if( privateNode.getParam("roadleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("road_leafsize parameter error");
				return false;
			}
			else{
				road_leafsize = fparam;
			}
		}

		if( privateNode.getParam("parkingleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("parking_leafsize parameter error");
				return false;
			}
			else{
				parking_leafsize = fparam;
			}
		}

		if( privateNode.getParam("sidewalkleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("sidewalk_leafsize parameter error");
				return false;
			}
			else{
				sidewalk_leafsize = fparam;
			}
		}

		if( privateNode.getParam("othergroundleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("otherground_leafsize parameter error");
				return false;
			}
			else{
				otherground_leafsize = fparam;
			}
		}

		if( privateNode.getParam("buildingleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("building_leafsize parameter error");
				return false;
			}
			else{
				building_leafsize = fparam;
			}
		}

		if( privateNode.getParam("fenceleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("fence_leafsize parameter error");
				return false;
			}
			else{
				fence_leafsize = fparam;
			}
		}

		if( privateNode.getParam("otherstructureleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("otherstructure_leafsize parameter error");
				return false;
			}
			else{
				otherstructure_leafsize = fparam;
			}
		}

		if( privateNode.getParam("lanemarkingleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("lanemarking_leafsize parameter error");
				return false;
			}
			else{
				lanemarking_leafsize = fparam;
			}
		}

		if( privateNode.getParam("vegetationleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("vegetation_leafsize parameter error");
				return false;
			}
			else{
				vegetation_leafsize = fparam;
			}
		}

		if( privateNode.getParam("trunkleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("trunk_leafsize parameter error");
				return false;
			}
			else{
				trunk_leafsize = fparam;
			}
		}

		if( privateNode.getParam("terrainleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("terrain_leafsize parameter error");
				return false;
			}
			else{
				terrain_leafsize = fparam;
			}
		}

		if( privateNode.getParam("poleleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("pole_leafsize parameter error");
				return false;
			}
			else{
				pole_leafsize = fparam;
			}
		}

		if( privateNode.getParam("trafficsignleafsize", fparam)){
			if(fparam < 0.0 ){
				ROS_ERROR("trafficsign_leafsize parameter error");
				return false;
			}
			else{
				trafficsign_leafsize = fparam;
			}
		}

		// Parameter from launch file


		return true;
	}

	void LaserMapping::get_tf_data(){
		//Look up transform between map and laserodometry
		while(true){
			try{
				listener.waitForTransform("map", reference_tf_frame_name, odom_time, ros::Duration(0.5) );
				listener.lookupTransform("map",  reference_tf_frame_name, odom_time, map_to_laserodometry );

				//ROS_INFO("GET TRANSFORM MAP TO KASERODOMETRY IN LASERMAPPING PROCESS");
				
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
			//person.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==40 && color_id.b==200){
			//bicyclist.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==30 && color_id.b==90){
			//motorcyclist.push_back(point);
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

		//std::cout << "End classify pointcloud" << std::endl;

	}

	void LaserMapping::convert_coordinate_of_pc(){

		pcl_ros::transformPointCloud("map", odom_time, velo_scans, reference_tf_frame_name, velo_scans, listener);
		//pcl_ros::transformPointCloud("map", odom_time, cloud_edge, "laserodometry", cloud_edge, listener);
		//pcl_ros::transformPointCloud("map", odom_time, cloud_centroid, "laserodometry", cloud_centroid, listener);

		//std::cout << "PointCloud is transformed from laserodometry frame to map frame" << std::endl;
	}

	void LaserMapping::reset_common_cloud(){

		velo_scans.clear();
		cloud_edge.clear();
		cloud_centroid.clear();

		//std::cout << "clear common cloud" << std::endl;
	}

	void LaserMapping::voxel_grid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in, const float semantic_leafsize){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_in) );

		//Do voxel grid filter
		if( semantic_leafsize != 0.0 ){
			//std::cout << "Do Voxel Grid Filter" << std::endl;

			pcl::VoxelGrid<pcl::PointXYZRGB> vg;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB> );

			vg.setInputCloud( pc );
			vg.setLeafSize(semantic_leafsize, semantic_leafsize, semantic_leafsize);

			vg.filter( *tmp );

			*pc = *tmp;
		}
		else{
			//std::cout << "No voxel grid" << std::endl;
		}

		//Contain pointcloud to pc_map_cloud
		for(size_t i=0; i< pc->points.size(); i++){
			pcl::PointXYZRGB point;
			
			point.r = pc->points[i].r;
			point.g = pc->points[i].g;
			point.b = pc->points[i].b;

			point.x = pc->points[i].x;
			point.y = pc->points[i].y;
			point.z = pc->points[i].z;

			pc_map_cloud.push_back(point);
		}

	}

	void LaserMapping::do_voxel_grid(){

		voxel_grid(unlabeled, unlabeled_leafsize);
		voxel_grid(outlier, outlier_leafsize);
		voxel_grid(car, car_leafsize);
		voxel_grid(bicycle, bicycle_leafsize);
		voxel_grid(bus, bus_leafsize);
		voxel_grid(motorcycle, motorcycle_leafsize);
		voxel_grid(onrails, onrails_leafsize);
		voxel_grid(truck, truck_leafsize);
		voxel_grid(othervehicle, othervehicle_leafsize);
		voxel_grid(road, road_leafsize);
		voxel_grid(parking, parking_leafsize);
		voxel_grid(sidewalk, sidewalk_leafsize);
		voxel_grid(otherground, otherground_leafsize);
		voxel_grid(building, building_leafsize);
		voxel_grid(fence, fence_leafsize);
		voxel_grid(otherstructure, otherstructure_leafsize);
		voxel_grid(lanemarking, lanemarking_leafsize);
		voxel_grid(vegetation, vegetation_leafsize);
		voxel_grid(trunk, trunk_leafsize);
		voxel_grid(terrain, terrain_leafsize);
		voxel_grid(pole, pole_leafsize);
		voxel_grid(trafficsign, trafficsign_leafsize);

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
		//person.clear();
		//bicyclist.clear();
		//motorcyclist.clear();
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

		pc_map_cloud.clear();

		//std::cout << "clear semantic cloud" << std::endl;
	}

	void LaserMapping::publish_pointcloud(){

		sensor_msgs::PointCloud2 pc_map;

		pcl::toROSMsg(pc_map_cloud, pc_map);

		pc_map.header.frame_id = "map";
		pc_map.header.stamp = odom_time;

		_pub_pc.publish(pc_map);

		_pub_odom.publish(odom_data);

		//std::cout << "Published PointCloud and Odometry data" << std::endl;

	}

	void LaserMapping::pointcloud_to_pcd(){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(pc_map_cloud) );

		std::string count_num = std::to_string(pcd_counter);

		//Define file path and file name
		//example:/home/username/catkin_ws/src/semantic_mesh_loam/PCD_data/semantic_mesh_loam123.pcd
		std::string path_ascii = file_path + file_name + "_ascii" + count_num +  ".pcd";
		std::string path_binary = file_path + file_name + "_binary" + count_num +  ".pcd";

		if( pcd_save_ascii == true ){
			pcl::io::savePCDFileASCII(path_ascii, *pcd_cloud);
			std::cout << "Save PointCloud data as " << path_ascii << std::endl;
		}

		if( pcd_save_binary == true ){
			pcl::io::savePCDFileBinary(path_binary, *pcd_cloud);
			std::cout << "Save PointCloud data as " << path_binary << std::endl;
		}

		pcd_counter += 1;

	}

	int LaserMapping::process(int counter){

		if(systemdelay > 0 ){
			--systemdelay;
			return 0;
		}

		Time start_sec = ros::Time::now();

		//get transform between map to laserodometry
		get_tf_data();

		//convert PointCloud's coordinate from laserodometry to map frame
		convert_coordinate_of_pc();

		//classify pointcloud
		classify_pointcloud();

		if( (counter % scan_counter) == 0 ){

            Time start_sec = ros::Time::now();

			//do voxel grid reducion per semantic point cloud
			do_voxel_grid();

			if(ros_publish_checker == true){
				//publish pointcloud as ROS msg
				publish_pointcloud();
			}

			if(pcd_save_checker == true){
				//publish pointcloud as .PCD file
				pointcloud_to_pcd();
			}
			
			//clear semantic point cloud like car and outlier
			reset_semantic_cloud();

            Time end_sec = ros::Time::now();

            double duration_time = end_sec.toSec() - start_sec.toSec();
            std::cout << "Process time:" << duration_time << std::endl;
		}

		//clear common pointcloud like
		reset_common_cloud();

		Time end_sec = ros::Time::now();

		double duration_process = end_sec.toSec() - start_sec.toSec();

		//std::cout << "process has ended" << std::endl;
		//std::cout << "process time: "<< duration_process << std::endl;

		return 1;

	}

	bool LaserMapping::check_status(){

		bool k;

		if(                velo_checker==true
				&& odom_checker==true
				&& edge_checker==true
				&& cent_checker==true){
			k = true;
		}
		else{
			k = false;
		}

		return k;
	}

	void LaserMapping::status_reset(){

		velo_checker = false;
		odom_checker = false;
		edge_checker = false;
		cent_checker = false;

	}

	void LaserMapping::spin(){

		ros::Rate rate(10);
		
		int counter = 0;

		while( ros::ok() ){

			ros::spinOnce();

			bool status_checker = check_status();

			if( status_checker==true ){
				
				int checker = process(counter);
				
				counter += checker;

				status_reset();
			}

			rate.sleep();
		}
	}

}

