#include"semantic_mesh_loam/laser_odometry.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "laser_odometry");
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");

	semloam::LaserOdometry laserodom;

	if(laserodom.setup(node, privateNode)){
		//Initial config has done
		laserodom.spin();

	}

	return 0;
}
