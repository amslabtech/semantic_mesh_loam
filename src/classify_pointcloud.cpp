#include"semantic_mesh_loam/classify_pointcloud.h"
#include"ros/ros.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "classify_pointcloud");
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	
	semloam::SemClassifer semclassifer;

	if(semclassifer.setup(node, privateNode)){

		ros::spin();
	}

	return 0;
}
