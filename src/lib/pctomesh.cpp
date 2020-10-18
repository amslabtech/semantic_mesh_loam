#include"semantic_mesh_loam/pc_to_mesh.h"

namespace semloam{

	PcToMesh::PcToMesh(void){

		std::cout << "Set up PcToMesh class" << std::endl;

	}

	bool PcToMesh::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

		float fparam;
		double dparam;
		int iparam;
		bool bparam;
		std::string strparam;

		_pubroad = node.advertise<sensor_msgs::PointCloud2>("/road",2);
		_pubcar = node.advertise<sensor_msgs::PointCloud2>("/car",2);


		if( privateNode.getParam("NormalSearchRadius", dparam) ){
			if(dparam < 0.0001){
				ROS_ERROR("Invalid NormalSearchRadius parameter");
				return false;
			}
			else{
				normal_search_radius = dparam;
			}
		}

		if( privateNode.getParam("filepath", strparam) ){
			if(strparam.length() < 1 ){
				ROS_ERROR("Invalid file path");
				return false;
			}
			else{
				file_path = strparam;
			}
		}

		if( privateNode.getParam("filename", strparam)){
			if(strparam.length() < 1){
				ROS_ERROR("Invalid file name");
				return false;
			}
			else{
				file_name = strparam;
			}
		}

		if( privateNode.getParam("filetype", strparam) ){
			if(strparam=="ascii" || strparam=="binary"){
				file_type = strparam;
			}
			else{
				ROS_ERROR("Invalid file type %s, You must describe file type ascii or binary in child charactor", strparam);
				return false;
			}
		}

		if( privateNode.getParam("PCLvisualizer", bparam) ){
			if( bparam==true || bparam==false ){
				pcl_mesh_visualize_checker = bparam;
			}
			else{
				ROS_ERROR("Invalid pcl_mesh_visualizer_checker parameter");
				return false;
			}
		}

		if( privateNode.getParam("ROSvisualizer", bparam) ){
			if(bparam==true || bparam==false){
				ros_mesh_visualize_checker = bparam;
			}
			else{
				ROS_ERROR("Invalid ros_mesh_visualizer_checker parameter");
				return false;
			}
		}


		std::cout << "Init set up is done" << std::endl;

		return true;
	}

	void PcToMesh::spin(){

		process();

	}

	void PcToMesh::process(){

		load_PCD();

		pcl::visualization::PCLVisualizer viewer;//create visualizer

		init_config_viewer_parameter(viewer);

		std::cout << "generate mesh" << std::endl;
		generate_mesh(viewer);

		config_tmp_viewer_parameter(viewer);

		while( ros::ok() ){

			viewer.spin();
			//publishing to ros msg is succeed
			//publish_rosmsg();

		}

	}

	void PcToMesh::publish_rosmsg(){
		sensor_msgs::PointCloud2 car_pc2, road_pc2;

		pcl::toROSMsg(car, car_pc2);
		pcl::toROSMsg(road, road_pc2);

		car_pc2.header.frame_id = "map";
		road_pc2.header.frame_id = "map";

		car_pc2.header.stamp = ros::Time::now();
		road_pc2.header.stamp = ros::Time::now();

		_pubroad.publish(road_pc2);
		_pubcar.publish(car_pc2);
	}

	void PcToMesh::load_PCD(){
		
		bool load_checker = true;

		while(load_checker==true){
			std::string number = std::to_string(load_counter);

			std::string file = file_path + file_name + "_" + file_type + number + ".pcd";

			int load_file = pcl::io::loadPCDFile(file, cloud_tmp);

			if(load_file == 0){//0 means loading pcd file is successfull
				load_counter += 1;

				classify_pointcloud();
				
				//cloud += cloud_tmp;
				cloud_tmp.clear();

				std::cout << "Loading " << file << " is successfull" << std::endl;

			}
			else{
				cloud_tmp.clear();
				std::cout << file << " is not found" << std::endl;

				load_checker = false;
			}
		}

		std::cout << "Loading PCD file has done" << std::endl;

	}

	void PcToMesh::classify_pointcloud(){
		size_t cloud_size = cloud_tmp.size();

		pcl::PointXYZRGB point;

		for(int i=0; i<cloud_size; i++){

			point.x = cloud_tmp[i].x;
			point.y = cloud_tmp[i].y;
			point.z = cloud_tmp[i].z;

			point.r = cloud_tmp[i].r;
			point.g = cloud_tmp[i].g;
			point.b = cloud_tmp[i].b;

			color_data color_id;
			color_id.r = int(point.r);
			color_id.g = int(point.g);
			color_id.b = int(point.b);

			if(!classify(point, color_id) ){
				//
			}
		}
	}

	bool PcToMesh::classify(const pcl::PointXYZRGB& point, const color_data& color_id){
		if(color_id.r==0 && color_id.g==0 && color_id.b==0){
			//unlabeled.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
			//outlier.push_back(point);
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

	void PcToMesh::init_config_viewer_parameter(pcl::visualization::PCLVisualizer& viewer){

		std::cout << "Config viewer init parameter except place" << std::endl;

		viewer.initCameraParameters();

	}

	void PcToMesh::generate_mesh(pcl::visualization::PCLVisualizer& viewer){

		std::cout << "Generate Triangle Mesh" << std::endl;
		
		if(car.size() > 10){
			generate_semantic_mesh(viewer, car, "car", 0.30, 400);
		}
		
		if(bicycle.size() > 10 ){
			generate_semantic_mesh(viewer, bicycle, "bicycle", 0.15, 200);
		}
		
		if(bus.size() > 10 ){
			generate_semantic_mesh(viewer, bus, "bus", 0.2, 300);
		}

		if(motorcycle.size() > 10 ){
			generate_semantic_mesh(viewer, motorcycle, "motorcycle", 0.25, 500);
		}
		
		if(onrails.size() > 10 ){
			generate_semantic_mesh(viewer, onrails, "onrails", 0.2, 400);
		}

		if(truck.size() > 10){
			generate_semantic_mesh(viewer, truck, "truck", 0.3, 600);
		}

		if(othervehicle.size() > 10 ){
			generate_semantic_mesh(viewer, othervehicle, "othervehicle", 0.2, 400);
		}

		if(parking.size() > 10 ){
			generate_semantic_mesh(viewer, parking, "parking", 0.35, 600);
		}

		if(sidewalk.size() > 10){
			generate_semantic_mesh(viewer, sidewalk, "sidewalk", 0.35, 600);
		}

		if(otherground.size() > 10){
			generate_semantic_mesh(viewer, otherground, "otherground", 0.35, 700);
		}

		if(building.size() > 10){
			generate_semantic_mesh(viewer, building, "building", 0.40, 900);
		}

		if(fence.size() > 10){
			generate_semantic_mesh(viewer, fence, "fence", 0.35, 900);
		}

		if(otherstructure.size() > 10 ){
			generate_semantic_mesh(viewer, otherstructure, "otherstructure", 0.35, 900);
		}

		if(lanemarking.size() > 10){
			generate_semantic_mesh(viewer, lanemarking, "lanemarking", 0.10, 200);
		}

		if(vegetation.size() > 10 ){
			generate_semantic_mesh(viewer, vegetation, "vegetation", 0.25, 600);
		}

		if(trunk.size() > 10){
			generate_semantic_mesh(viewer, trunk, "trunk", 0.10, 600);
		}

		if(terrain.size() > 10){
			generate_semantic_mesh(viewer, terrain, "terrain", 0.20, 500);
		}

		if(pole.size() > 10){
			generate_semantic_mesh(viewer, pole, "pole", 0.10, 500);
		}

		if(trafficsign.size() > 10){
			generate_semantic_mesh(viewer, trafficsign, "trafficsign", 0.10, 500);
		}

		if(road.size() > 10 ){
			generate_semantic_mesh(viewer, road, "road", 0.40, 800);
		}

	}

	void PcToMesh::generate_semantic_mesh(pcl::visualization::PCLVisualizer& viewer,const pcl::PointCloud<pcl::PointXYZRGB>& semantic_cloud, const std::string semantic_name, double search_radius, double max_neighbor){
		
		std::cout << "Create semantic mesh of " << semantic_name << " " << std::endl;
		std::cout << "compute normal of " << semantic_name << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGB>(semantic_cloud));
		//create smart pointer because normal estimation and greedytriangulation need smart pointer

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

		kdtree->setInputCloud(cloudin);

		//Set search method
		ne.setSearchMethod(kdtree);

		//Set normal search radius
		ne.setRadiusSearch(normal_search_radius);

		//表裏を決める部分
		ne.setViewPoint(0, 0, 0);

		ne.setInputCloud(cloudin);

		ne.compute(*cloud_normal);
		
		//アホみたいな話だがPointNormalで受け取ってもXYZの情報は0のままなので写してやる必要がある
		for (int i = 0; i < cloud_normal->points.size(); i++){

			cloud_normal->points[i].x = cloudin->points[i].x;
			cloud_normal->points[i].y = cloudin->points[i].y;
			cloud_normal->points[i].z = cloudin->points[i].z;

		}

		/*
		std::cout << cloudin->points[0].x << std::endl;
		std::cout << cloudin->points[0].y << std::endl;
		std::cout << cloudin->points[0].z << std::endl;
		*/

		std::cout << "Compute polygon mesh of " << semantic_name << std::endl;

		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh triangles;

		//微調整が必要なパラメータ
		gp3.setSearchRadius(search_radius);
		gp3.setMu(5.0);
		gp3.setMaximumNearestNeighbors(max_neighbor);
		gp3.setMaximumSurfaceAngle(PI/2.0);
		gp3.setMinimumAngle(-PI);
		gp3.setMaximumAngle(PI);
		gp3.setNormalConsistency(true);

		gp3.setInputCloud(cloud_normal);
		
		//Create new KdTree for PointNormal
		pcl::search::KdTree<pcl::PointNormal>::Ptr treePN(new pcl::search::KdTree<pcl::PointNormal>);
		treePN->setInputCloud(cloud_normal);
		gp3.setSearchMethod(treePN);

		//COmpute triangles
		gp3.reconstruct(triangles);

		viewer.addPolygonMesh(triangles, semantic_name);
		
		
		//Convert RGB data 0~255 to 0.0~1.0
		float color_r = float(cloudin->points[0].r)/255.0;
		float color_g = float(cloudin->points[0].g)/255.0;
		float color_b = float(cloudin->points[0].b)/255.0;

		//viewer.addPointCloud<pcl::PointXYZRGB>(cloudin, semantic_name, v1);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, semantic_name, v1);

		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, semantic_name);

		std::cout << "Semantic mesh " << semantic_name << " was added to PCL visualizer" << std::endl;

	}

	void PcToMesh::config_tmp_viewer_parameter(pcl::visualization::PCLVisualizer& viewer){

		viewer.setCameraPosition(0.0,0.0,0.0, 0.0,0.0,0.0);// x, y, z, roll, pitch, yaw

	}




}
