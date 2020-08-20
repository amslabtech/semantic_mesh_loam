#ifndef __CLASSIFY_POINTCLOUD_H
#define __CLASSIFY_POINTCLOUD_H

#include"util.h"
#include"ros/ros.h"
#include"sensor_msgs/PointCloud2.h"
#include"pcl_ros/point_cloud.h"

class SemClassifer{

	private:
		ros::NodeHandle nh;

		sensor_msgs::PointCloud2 segmented_points;

		pcl::PointCloud<pcl::PointXYZRGB> lasercloudin:

		semantic_dictionary dict[34];

		ros::Subscriber velo_sub;
		ros::Publisher classified_pub;

	public:
		SemClassifer(viod);
		void process();
		void colored_point_callback(const sensor_msgs::PointCloud2i::Constptr& msg);

