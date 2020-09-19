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
#include"pcl/registration/icp.h"

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

			bool initialization();

			bool hasNewData();

			void get_relative_trans();

			void get_tf_data();

			void convert_coordinate_of_pc();

			Eigen::Matrix4f init_pc_slide();

			Eigen::Matrix4f pcl_pc_slide();

			void send_tf_data(Eigen::Matrix4f Tm);

			void final_transform_pc(Eigen::Matrix4f laserodometry_trans_matrix);

			void publish_result();

			void reset();

		private:
			int scancount = 0;
			float scanperiod;
			uint16_t ioratio;
			long framecount;
			size_t max_iterations;
			bool system_initialized_checker;
			int __systemdelay = 10;

			//ICP parameter
			float MaxCorrespondDistance = 0.15;
			int MaximumIterations = 50;
			float TransformationEpsilon = 1e-8;
			float EuclideanFitnessEpsilon = 1.0;

			float delta_t_abort; //optimization abort threshold for delta T
			float delta_r_abort; //optimization abort threshold for delta R

			pcl::PointCloud<pcl::PointXYZRGB> velo_scans;
			pcl::PointCloud<pcl::PointXYZRGB> CloudCentroid;
			pcl::PointCloud<pcl::PointXYZRGB> CloudEdge;
			pcl::PointCloud<pcl::PointXYZRGB> FeatureCloud;

			//Contain last scan's feature points data
			//pcl::PointCloud<pcl::PointXYZRGB> _lastCloudCentroid;
			//pcl::PointCloud<pcl::PointXYZRGB> _lastCloudEdge;
			pcl::PointCloud<pcl::PointXYZRGB> _lastFeatureCloud;

			pcl::PointCloud<pcl::PointXYZRGB> tmp_pc_stored;

			//KdTree
			//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _lastCloudCentroidTree;
			//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _lastCloudEdgeTree;

			//Contain Centroid index
			//std::vector<int> CloudCentroidInd;
			//std::vector<int> _lastCloudCentroidInd;

			//Contain Edge index
			//std::vector<int> CloudEdgeInd;
			//std::vector<int> _lastCloudEdgeInd;

			nav_msgs::Odometry odom_data; //Contain current odometry data
			nav_msgs::Odometry _last_odom_data; //Constain last scans odometry data
			
			pos_trans relative_pos_trans;

			nav_msgs::Odometry laserodometry; //Calibrated odometry data
			
			tf::StampedTransform odometrytrans;
			tf::StampedTransform velo_to_map;
			tf::StampedTransform laserodometry_to_map;

			tf::TransformBroadcaster br; //publish tf data between map and laserodometry

			Time velo_scans_time;
			Time CloudCentroid_time;
			Time CloudEdge_time;
			Time odom_data_time;

			Time _last_CloudEdge_time;
			Time _last_CloudCentroid_time;
			Time _last_odom_data_time;

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
			//ros::Publisher _pubFeaturePoints3;
			ros::Publisher _pubVelodynePoints3;

			ros::Subscriber _subCentroid;
			ros::Subscriber _subEdge;
			ros::Subscriber _subVelodynePoints;
			ros::Subscriber _subOdometry;

			tf::TransformListener listener;

	};

}





#endif
