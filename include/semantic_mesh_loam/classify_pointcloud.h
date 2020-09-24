#ifndef __CLASSIFY_POINTCLOUD_H
#define __CLASSIFY_POINTCLOUD_H

#include"util.h"
#include"sensor_msgs/PointCloud2.h"
#include"pcl/point_cloud.h"
#include<stdint.h>
#include"ros/node_handle.h"
#include<utility>
#include<vector>
#include"pcl/point_types.h"
#include"pcl_conversions/pcl_conversions.h"
#include"pcl/segmentation/extract_clusters.h"
#include"pcl/filters/extract_indices.h"
#include"pcl/features/normal_3d.h"
//#include"pcl/visualization/cloud_viewer.h"
#include"nav_msgs/Odometry.h"
#include"pcl/features/normal_3d_omp.h"

namespace semloam{



	class MultiScanMapper{

		public:
			MultiScanMapper(
					const float& lowerBound = -15,
					const float& upperBound = 15,
					const uint16_t& nScanRings = 16);

			const float& getLowerBound(){
				return _lowerBound;
			}
			const float& getUpperBound(){
				return _upperBound;
			}
			const uint16_t& getNumberOfScanRings(){
				return _nScanRings;
			}

			void set(       const float& lowerBound,
					const float& upperBound,
					const uint16_t& nScanRings);

			int getRingForAngle(const float& angle);

			static inline MultiScanMapper Velodyne_VLP_16(){
				return MultiScanMapper(-15, 15, 16);
			}
			static inline MultiScanMapper Velodyne_HDL_32(){
				return MultiScanMapper(-30.67f, 10.67f, 32);
			}
			static inline MultiScanMapper Velodyne_HDL_64E(){
				return MultiScanMapper(-24.9f, 2, 64);
			}


		private:
			float _lowerBound;
			float _upperBound;
			uint16_t _nScanRings;
			float _factor;


	};

	class RegistrationParams{
		public:
			RegistrationParams(const float& scanperiod_ = 0.1,
					const int& imuhistorysize_ = 200,
					const int& odomhistorysize_ = 200,
					const int& nfeatureregions_ = 6,
					const int& curvatureregion_ = 5,
					const int& maxcornersharp_ = 4,
					const float& lessflatfiltersize_ = 0.2,
					const float& surfacecurvaturethreshold_ = 0.1);
			
			
			float scanperiod;

			int imuhistorysize;

			int odomhistorysize;

			int nfeatureregions;

			int curvatureregion;

			int maxcornersharp;

			float lessflatfiltersize;

			float surfacecurvaturethreshold;


	};

	class SemClassifer{
		
		private:
			//pcl::visualization::PCLVisualier viewer {"Euclidean Clustering"};
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> empty_vec;

			double cluster_torelance;
			int min_cluster_size;

			const size_t cluster_size = 75000;

		public:
			int Clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
			// void visualization();

		public:
			SemClassifer();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &laserscan);

			void odometry_callback(const nav_msgs::OdometryConstPtr& odom);

			bool setParams(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);
			bool parseParams(ros::NodeHandle& privateNode, RegistrationParams& config_out);

			void publish_data(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const pcl::PointCloud<pcl::PointXYZRGB>& CloudCentroid, const pcl::PointCloud<pcl::PointXYZRGB>& CloudEdge, const Time& scanTime);

		private:

			bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);

			bool configure(const RegistrationParams& config = RegistrationParams());

			void process(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const Time& scanTime);

			bool classify(const pcl::PointXYZRGB& point, const color_data& color_id);

			void extract_centroid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
			void calc_ave_point(int cluster_num);

			void extract_edge_point(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
			void normal_edge_process(int cluster_num);
			void normal_edge_process_OMP(int cluster_num);
			void extract_edge_point_normal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

			float searchradius;
			float curvaturethreshold;


		private:

			//RegistrationParams _config;
			//CircularBuffer<IMUstate> _imuhistory;
			//CircularBuffer<Odomstate> _odomhistory;
			//
			int _systemDelay = 3;

			const size_t pc_size_big = 100000;
			const size_t pc_size_mid =  50000;
			const size_t pc_size_min =  10000;

			ros::Subscriber _subLaserCloud;
			ros::Subscriber _subOdometry;

			ros::Publisher _pubLaserCloud;
			ros::Publisher _pubCentroid;
			ros::Publisher _pubEdge;
			ros::Publisher _pubOdom;

			MultiScanMapper _scanMapper;

			nav_msgs::Odometry odom_data;
			bool odom_checker = false;

			pcl::PointCloud<pcl::PointXYZRGB> CloudCentroid;
			pcl::PointCloud<pcl::PointXYZRGB> CloudEdge;

			pcl::PointCloud<pcl::PointXYZRGB> unlabeled;
			pcl::PointCloud<pcl::PointXYZRGB> outlier;
			pcl::PointCloud<pcl::PointXYZRGB> car;
			pcl::PointCloud<pcl::PointXYZRGB> bicycle;
			pcl::PointCloud<pcl::PointXYZRGB> bus;
			pcl::PointCloud<pcl::PointXYZRGB> motorcycle;
			pcl::PointCloud<pcl::PointXYZRGB> onrails;
			pcl::PointCloud<pcl::PointXYZRGB> truck;
			pcl::PointCloud<pcl::PointXYZRGB> othervehicle;
			pcl::PointCloud<pcl::PointXYZRGB> person;
			pcl::PointCloud<pcl::PointXYZRGB> bicyclist;
			pcl::PointCloud<pcl::PointXYZRGB> motorcyclist;
			pcl::PointCloud<pcl::PointXYZRGB> road;
			pcl::PointCloud<pcl::PointXYZRGB> parking;
			pcl::PointCloud<pcl::PointXYZRGB> sidewalk;
			pcl::PointCloud<pcl::PointXYZRGB> otherground;
			pcl::PointCloud<pcl::PointXYZRGB> building;
			pcl::PointCloud<pcl::PointXYZRGB> fence;
			pcl::PointCloud<pcl::PointXYZRGB> otherstructure;
			pcl::PointCloud<pcl::PointXYZRGB> lanemarking;
			pcl::PointCloud<pcl::PointXYZRGB> vegetation;
			pcl::PointCloud<pcl::PointXYZRGB> trunk;
			pcl::PointCloud<pcl::PointXYZRGB> terrain;
			pcl::PointCloud<pcl::PointXYZRGB> pole;
			pcl::PointCloud<pcl::PointXYZRGB> trafficsign;

	};



}


#endif
