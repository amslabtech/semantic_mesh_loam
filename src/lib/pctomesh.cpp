#include"semantic_mesh_loam/pc_to_mesh.h"

namespace semloam{

	PcToMesh::PcToMesh(void){

		std::cout << "Set up PcToMesh class" << std::endl;

	}

	bool PcToMesh::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

		float fparam;
		int iparam;
		bool bparam;
		std::string strparam;

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

		generate_mesh();

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



}
