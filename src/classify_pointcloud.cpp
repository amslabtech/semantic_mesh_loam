#include"semantic_mesh_loam/classify_pointcloud.h"
#include"ros/ros.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "classify_pointcloud");
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	
	semloam::SemClassifer semclassifer;

	std::cout << "start" << std::endl;

	ros::Rate rate(10);

	if(semclassifer.setup(node, privateNode)){

		ros::spin();

		rate.sleep();
	}

	std::cout << "end classify program" << std::endl;
	return 0;
}
