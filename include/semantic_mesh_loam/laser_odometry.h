#ifndef __LASER_ODOMETRY_H
#define __LASER_ODOMETRY_H

#include<vector>
#include<utility>

#include"util.h"

#include"ros/node_handle.h"
#include"pcl_ros/transforms.h"
#include"sensor_msgs/PointCloud2.h"
#include"nav_msgs/Odometry.h"
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"
#include"geometry_msgs/Pose.h"
#include"pcl_conversions/pcl_conversions.h"
#include"pcl/segmentation/extract_clusters.h"

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

			void process();

			bool hasNewData();

			void get_tf_data();

			void convert_coordinate_of_pc();

			//void init_pc_slide();

		private:
			int scancount = 0;
			float scanperiod;
			uint16_t ioratio;
			long framecount;
			size_t max_iterations;
			bool system_initialized_checker;

			float delta_t_abort; //optimization abort threshold for delta T
			float delta_r_abort; //optimization abort threshold for delta R

			pcl::PointCloud<pcl::PointXYZRGB> velo_scans;
			pcl::PointCloud<pcl::PointXYZRGB> CloudCentroid;
			pcl::PointCloud<pcl::PointXYZRGB> CloudEdge;

			//Contain last scan's feature points data
			pcl::PointCloud<pcl::PointXYZRGB> _lastCloudCentroid;
			pcl::PointCloud<pcl::PointXYZRGB> _lastCloudEdge;

			//KdTree
			//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _lastCloudCentroidTree;
			//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _lastCloudEdgeTree;

			//Contain Centroid index
			std::vector<int> CloudCentroidInd;
			std::vector<int> _lastCloudCentroidInd;

			//Contain Edge index
			std::vector<int> CloudEdgeInd;
			std::vector<int> _lastCloudEdgeInd;

			nav_msgs::Odometry odom_data; //Contain current odometry data
			nav_msgs::Odometry _last_odom_data; //It may be unneccesary

			nav_msgs::Odometry laserodometry; //Calibrated odometry data
			
			tf::StampedTransform laserodometrytrans;
			tf::StampedTransform velo_to_map;

			Time velo_scans_time;
			Time CloudCentroid_time;
			Time CloudEdge_time;
			Time odom_data_time;

			Time _last_CloudEdge_time;
			Time _last_CloudCentroid_time;

			bool velo_scans_checker = false;
			bool CloudCentroid_checker = false;
			bool CloudEdge_checker = false;
			bool odom_data_checker = false;

			const size_t scan_size = 150000;
			const size_t feature_size = 10000;
			const size_t ind_size = 5000;

			ros::Publisher _pubLaserOdomToInit;
			ros::Publisher _pubCentroidPointLast;
			ros::Publisher _pubEdgePointLast;
			ros::Publisher _pubVelodynePoints3;

			ros::Subscriber _subCentroid;
			ros::Subscriber _subEdge;
			ros::Subscriber _subVelodynePoints;
			ros::Subscriber _subOdometry;

			tf::TransformListener listener;

	};

}





#endif
