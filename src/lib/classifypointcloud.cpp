#include"semantic_mesh_loam/classify_pointcloud.h"

#include<pcl_conversions/pcl_conversions.h>

namespace semloam{

	MultiScanMapper::MultiScanMapper(const float& lowerBound, const float& upperBound, const uint16_t&nScanRings)
		:_lowerBound(lowerBound),
		 _upperBound(upperBound),
		 _nScanRings(nScanRings),
		 _factor( (nScanRings -1 ) / (upperBound - lowerBound) ){}
	

	void MultiScanMapper::set(const float& lowerBound, const float& upperBound, const uint16_t& nScanRings){
		_lowerBound = lowerBound;
		_upperBound = upperBound;
		_nScanRings = nScanRings;
		_factor = (nScanRings -1 ) / (upperBound - lowerBound);
	}

	bool SemClassifer::parseParams(ros::NodeHandle& privateNode, RegistrationParams& config_out){
		bool success = true;
		int iParam = 0;
		float fParam = 0.0;

		if(privateNode.getParam("scanperiod",fParam)){
			if(fParam <= 0){
				ROS_ERROR("Invalid scanPeriod parameter");
				success = false;
			}
			else{
				config_out.scanperiod = fParam;
				ROS_INFO("Set scanperiod %g", fParam);
			}
		}
		if(privateNode.getParam("imuhistorysize",iParam)){
			if(iParam < 1){
				ROS_ERROR("Invalid imuhistory parameter");
				success = false;
			}
			else{
				config_out.imuhistorysize = iParam;
				ROS_INFO("Set imuhistory %d", iParam);
			}
		}

		if(privateNode.getParam("odomhistorysize",iParam)){
			if(iParam < 1){
				ROS_ERROR("Invalid odomhistory parameter");
				success = false;
			}
			else{
				config_out.odomhistorysize = iParam;
				ROS_INFO("Set odomhistorysize %d", iParam);
			}
		}
		if(privateNode.getParam("nfeatureregions",iParam)){
			if(iParam < 1){
				ROS_ERROR("Invalid nfeatureregions parameter");
				success = false;
			}
			else{
				config_out.nfeatureregions = iParam;
				ROS_INFO("Set nfeatureregions %d", iParam);
			}
		}
		if(privateNode.getParam("curvatureregion",iParam)){
			if(iParam < 1){
				ROS_ERROR("Invalid curvatureregion parameter");
				success = false;
			}
			else{
				config_out.curvatureregions = iParam;
				ROS_INFO("Set curvatureregion %d", iParam);
			}
		}

		if(privateNode.getParam("maxcornersharp",iParam)){
			if(iParam < 1){
				ROS_ERROR("Invalid maxcornersharp parameter");
				success = false;
			}
			else{
				config_out.maxcornersharp = iParam;
				ROS_INFO("Set maxcornersharp %d", iParam);
			}
		}

		if(privateNode.getParam("surfacecurvaturethreshold",fParam)){
			if(fParam <= 0.001){
				ROS_ERROR("Invalid surfacecurvaturethreshold parameter");
				success = false;
			}
			else{
				config_out.surfacecurvaturethreshold = fParam;
				ROS_INFO("Set surfacecurvaturethreshold %g", fParam);
			}
		}

		if(privateNode.getParam("lessflatfiltersize",fParam)){
			if(fParam <= 0.001){
				ROS_ERROR("Invalid lessflatfiltersize parameter");
				success = false;
			}
			else{
				config_out.lessflatfiltersize = fParam;
				ROS_INFO("Set lessflatfiltersize %g", fParam);
			}
		}

		return success;
	}


	bool SemClassifer::setParams(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out){
		if(!parseParams(privateNode, config_out)){
			return false;
		}

		_pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points2", 2);
		_pubCentroid = node.advertise<sensor_msgs::PointCloud2>("/centroid_points", 2);
		_pubEdge = node.advertise<sensor_msgs::PointCloud2>("/edge_points", 2);

		return true;
	}

	bool SemClassifer::configure(const RegistrationParams& config){
		//_config = config;
		//_imuhistory.ensureCapacity(_config.imuhistory);
		//_odomhistory.ensureCapacity(_config.odomhistorysize);

		return true;
	}


	SemClassifer::SemClassifer(const MultiScanMapper& scanMapper)
		: _scanMapper(scanMapper){};

	bool SemClassifer::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

		RegistrationParams config;

		if(!setupROS(node, privateNode, config)){
			return false;
		}

		configure(config);

		return true;
	}

	bool SemClassifer::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config){

		if(!setParams(node, privateNode, config)){
				return false;
		}

		std::string lidarName;

		if(privateNode.getParam("lidar", lidarName)){
			if(lidarName == "VLP-16"){
				_scanMapper = MultiScanMapper::Velodyne_VLP_16();
			}
			else if(lidarName == "HDL-32"){
				_scanMapper = MultiScanMapper::Velodyne_HDL_32();
			}
			else if(lidarName == "HDL-64E"){
				_scanMapper = MultiScanMapper::Velodyne_HDL64E();
			}
			else{
				ROS_ERROR("Invalid lidar parameter");
				return false;
			}

			ROS_INFO("Set %s scan mappe.", lidarName.c_str());

			if(!privateNode.hasParam("scanPeriod")){
				config.scanperiod = 0.1;

				ROS_INFO("Set scan period: %f", config.scanperiod);
			}
		}
		else{
			float vAngleMin, vAngleMax;
			int nScanRings;

			if(             privateNode.getParam("maxVerticalAngle", vAngleMin) &&
					privateNode.getParam("maxVerticalAngle", vAngleMax) &&
					privateNode.getParam("nScanRings", nScanRings) ){
				if(vAngleMIn >= vAngleMax ){
					ROS_ERROR("Invalid vertical range");
					return false;
				}
				else if(nScanRings < 2){
					ROS_ERROR("Invalid number of scan rings(n < 2)");
					return false;
				}
			}

			_scanMapper.set(vAngleMin, vAngleMax, nScanRings);
			ROS_INFO("Set scan mapper");
		}

		_subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
			("/Velodyne_points", 2, &SemClassifer::pointcloud_callback, this);

		return true;
	}


				
}