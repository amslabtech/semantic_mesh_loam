#include"semantic_mesh_loam/laser_mapping.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "laser_mapping");

	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");

	semloam::LaserMapping lasermapping;

	if( lasermapping.setup(node, privateNode) ){

		lasermapping.spin();

	}

	return 0;
}
