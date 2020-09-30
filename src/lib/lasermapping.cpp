#include"semantic_mesh_loam/laser_mapping.h"

namespace semloam{

	LaserMapping::LaserMapping(){
		//korekara
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

		// Parameter from launch file


		return true;
	}

	

}
