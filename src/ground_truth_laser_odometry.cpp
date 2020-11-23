#include "semantic_mesh_loam/ground_truth_laser_odometry.h"

int main(int argc, char** argv){
    
    ros::init(argc, argv, "ground_truth_laser_odometry");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semloam::GT_LaserOdometry laserodom;

    if(laserodom.setup(node, privateNode)){

        laserodom.spin();

    }

    return 0;
}
