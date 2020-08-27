#ifndef __CLASSIFY_POINTCLOUD_H
#define __CLASSIFY_POINTCLOUD_H

#include"util.h"
#include"ros/ros.h"
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
#include"pcl/visualization/cloud_viewer.h"

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
			static inline MultiScanMapper Velodyne_VLP_32(){
				return MultiScanMapper(-30.67f, 10.67f, 32);
			}
			static inline MultiScanMapper Velodyne_VLP_64E(){
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

		public:
			SemClassifer(const MultiScanMapper& scanMapper = MultiScanMapper());

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &laserscan);

			bool setParams(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);
			bool parseParams(ros::NodeHandle& privateNode, RegistrationParams& config_out);

		private:

			bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);

			bool configure(const TegistrationParams& config = RegistrationParams());

			void process(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const Time& scanTime);

			void classify(const pcl::PointXYZRGB& point, const int& color_id);

		private:

			//RegistrationParams _config;
			//CircularBuffer<IMUstate> _imuhistory;
			//CircularBuffer<Odomstate> _odomhistory;

			ros::Subscriber _subLaserCloud;

			ros::Publisher _pubLaserCloud;
			ros::Publisher _pubCentroid;
			ros::Publisher _pubEdge;

			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> unlabeled;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> outlier;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> car;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> bicycle;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> bus;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> motorcycle;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> onrails;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> truck;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> othervehicle;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> person;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> bicyclist;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> motorcyclist;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> road;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> parking;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> sidewalk;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> otherground;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> building;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> fence;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> otherstructure;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> lanemarking;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> vegetation;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> trunk;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> terrain;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pole;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> trafficsign;

	};



}
