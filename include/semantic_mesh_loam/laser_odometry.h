#ifndef __LASER_ODOMETRY_H
#define __LASER_ODOMETRY_H

#include<vector>
#include<utility>

#include"util.h"

#include"ros/node_handle.h"
#include"sensor_msgs/PointCloud2.h"
#include"nav_msgs/Odometry.h"
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"

namespace semloam{

	class LaserOdometry{

		public:
			LaserOdometry();
			
			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void spin();

			void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata);

			void centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata);

			void edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata);

			void odometry_callback(const nav_msgs::OdometryConstPtr& odomdata);

		private:
			float scanperiod;
			uint16_t ioratio;
			long framecount;
			size_t max_iterations;
			bool system_initialized_checker;

			float delta_t_abort; //optimization abort threshold for delta T
			float delta_r_abort; //optimization abort threshold for delta R

			pcl::PointCloud<pcl::PointXYZRGB> velo_scans;
			const size_t scan_size = 150000;

			pcl::PointCloud<pcl::PointXYZRGB> CloudCentroid;
			pcl::PointCloud<pcl::PointXYZRGB> CloudEdge;
			const size_t feature_size = 5000;

			ros::Publisher _pubLaserOdomToInit;
			ros::Publisher _pubCentroidPointLast;
			ros::Publisher _pubEdgePointLast;
			ros::Publisher _pubVelodynePoints3;

			ros::Subscriber _subCentroid;
			ros::Subscriber _subEdge;
			ros::Subscriber _subVelodynePoints;
			ros::Subscriber _subOdometry;

	};

}





#endif
