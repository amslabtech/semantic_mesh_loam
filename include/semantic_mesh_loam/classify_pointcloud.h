#ifndef __CLASSIFY_POINTCLOUD_H
#define __CLASSIFY_POINTCLOUD_H

#include"util.h"
#include"ros/ros.h"
#include"sensor_msgs/PointCloud2.h"
#include"pcl_ros/point_cloud.h"

namespace semloam{

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
			SemClassifer();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &laserscan);

			bool setParams(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);
			bool parseParams(ros::NodeHandle& privateNode, RegistrationParams& config_out);

		private:

			bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);

			bool configure(const TegistrationParams& config = RegistrationParams());

			void process();

		private:

			//RegistrationParams _config;
			//CircularBuffer<IMUstate> _imuhistory;
			//CircularBuffer<Odomstate> _odomhistory;

			ros::Subscriber _subLaserCloud;

			ros::Publisher _pubLaserCloud;
			ros::Publisher _pubCentroid;
			ros::Publisher _pubEdge;



	};



}
