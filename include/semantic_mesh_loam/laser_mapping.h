#ifndef __LASER_MAPPING_H
#define __LASER_MAPPING_H

#include"ros/ros.h"
#include"ros/node_handle.h"

#include"util.h"

#include"sensor_msgs/PointCloud2.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/Pose.h"

#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"

#include"pcl_pointcloud.h"
#include"pcl/point_types.h"
#include"pcl_conversions/pcl_conversions.h"

#include"pcl_ros/transforms.h"

#include"pcl/filters/voxel_grid.h"

namespace semloam{

	class LaserMapping{

		public:
			LaserMapping();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void odometry_callback(const nav_msgs::OdometryConstPtr& odomdata);

			void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata);

			void centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata);

			void edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata);

			void spin();


		private:
			ros::Subscriber _sub_odometry;
			ros::Subscriber _sub_velodyne;
			ros::Subscriber _sub_centroid;
			ros::Subscriber _sub_edge;

			pcl::PointCloud<pcl::PointXYZRGB> velo_scans;
			pcl::PointCloud<pcl::PointXYZRGB> cloud_centroid;
			pcl::PointCloud<pcl::PointXYZRGB> cloud_edge;

			nav_msgs::Odometry odom_data;


}

#endif
