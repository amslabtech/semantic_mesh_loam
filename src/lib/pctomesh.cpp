#include"semantic_mesh_loam/pc_to_mesh.h"

namespace semloam{

	PcToMesh::PcToMesh(void){

		std::cout << "Set up PcToMesh class" << std::endl;

	}

	bool PcToMesh::PcToMesh(ros::NodeHandle& node, ros::NodeHandle& privateNode){

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



}
