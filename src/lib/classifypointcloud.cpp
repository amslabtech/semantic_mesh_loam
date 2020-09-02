#include"semantic_mesh_loam/classify_pointcloud.h"

namespace semloam{

	RegistrationParams::RegistrationParams(const float& scanPeriod_,
                                       const int& imuHistorySize_,
                                       const int& nFeatureRegions_,
                                       const int& curvatureRegion_,
                                       const int& maxCornerSharp_,
                                       const int& maxSurfaceFlat_,
                                       const float& lessFlatFilterSize_,
                                       const float& surfaceCurvatureThreshold_)
    : scanperiod(scanPeriod_),
      imuhistorysize(imuHistorySize_),
      nfeatureregions(nFeatureRegions_),
      curvatureregion(curvatureRegion_),
      maxcornersharp(maxCornerSharp_),
      lessflatfiltersize(lessFlatFilterSize_),
      surfacecurvaturethreshold(surfaceCurvatureThreshold_)
{};


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
				config_out.curvatureregion = iParam;
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


	SemClassifer::SemClassifer(){
		_scanMapper = MultiScanMapper();
		
		cluster_torelance = 0.1;
		min_cluster_size = 10;

		searchradius = 0.3;//Radius used to calculate point normal tekitou
		curvaturethreshold = 0.2; //tekitou 
	}

	bool SemClassifer::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

		RegistrationParams config;

		std::cout << "setup" <<std::endl;

		if(!setupROS(node, privateNode, config)){
			return false;
		}

		configure(config);

		return true;
	}

	void SemClassifer::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &laserscan){

		if(_systemDelay > 0){
			--_systemDelay;
			return;
		}


		pcl::PointCloud<pcl::PointXYZRGB> laserCloudIn;
		pcl::fromROSMsg( *laserscan, laserCloudIn);

		std::cout << "laser" << std::endl;
		//std::cout << laserCloudIn.points[0].x << std::endl;

		//process(laserCloudIn, fromROSTime(laserscan->header.stamp));
		process(laserCloudIn, laserscan->header.stamp);

	}

	bool SemClassifer::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config){

		std::cout << "setup ros" << std::endl;
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
				_scanMapper = MultiScanMapper::Velodyne_HDL_64E();
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
				if(vAngleMin >= vAngleMax ){
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
		

		std::cout << "param" <<std::endl;

		_subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
			("/velodyne_points", 2, &SemClassifer::pointcloud_callback, this);

		return true;
	}

	bool SemClassifer::classify(const pcl::PointXYZRGB& point, const int& color_id){

		/*char id[8];
		snprintf(id, 8, "%x", color_id);*/

		int id = color_id;

		//std::cout << "%x" << id << std::endl;
		if(id == std::stoi("0x000000", nullptr, 16)){
			unlabeled.push_back(point);
		}
		else if(id == 0xff0000){
			outlier.push_back(point);
		}
		else if(id == std::stoi("0x6496f5", nullptr, 16)){
			car.push_back(point);
			std::cout << "car" << std::endl;
		}
		else if(id == 0x64e6f5){
			bicycle.push_back(point);
		}
		else if(id == 0x6450fa){
			bus.push_back(point);
		}
		else if(id == 0x1e3c96){
			motorcycle.push_back(point);
		}
		else if(id == 0x0000ff){
			onrails.push_back(point);
		}
		else if(id == 0x501eb4){
			truck.push_back(point);
		}
		else if(id == 0xff1e1e){
			person.push_back(point);
		}
		else if(id == 0xff28c8){
			bicyclist.push_back(point);
		}
		else if(id == 0x961e5a){
			motorcyclist.push_back(point);
		}
		else if(id == 0xff00ff){
			road.push_back(point);
		}
		else if(id == 0xff96ff){
			parking.push_back(point);
		}
		else if(id == 0x4b004b){
			sidewalk.push_back(point);
		}
		else if(id == 0xaf004b){
			otherground.push_back(point);
		}
		else if(id == 0xffc800){
			building.push_back(point);
		}
		else if(id == 0xff7832){
			fence.push_back(point);
		}
		else if(id == 0xff9600){
			otherstructure.push_back(point);
		}
		else if(id == 0x96ffaa){
			lanemarking.push_back(point);
		}
		else if(id == 0x00af00){
			vegetation.push_back(point);
		}
		else if(id == 0x873c00){
			trunk.push_back(point);
		}
		else if(id == 0x96f050){
			terrain.push_back(point);
		}
		else if(id == 0xfff096){
			pole.push_back(point);
		}
		else if(id == 0xff0000){
			trafficsign.push_back(point);
		}

		//std::cout << "classify" << std::endl;
		return true;
	}

	void SemClassifer::publish_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const pcl::PointCloud<pcl::PointXYZRGB>& CloudCentroid, const pcl::PointCloud<pcl::PointXYZRGB>& CloudEdge, const Time& scanTime){
		sensor_msgs::PointCloud2 velo, cent, edge;

		pcl::toROSMsg(laserCloudIn, velo);
		pcl::toROSMsg(CloudCentroid, cent);
		pcl::toROSMsg(CloudEdge, edge);

		velo.header.frame_id = "vehicle";
		velo.header.stamp = scanTime;

		cent.header.frame_id = "vehicle";
		cent.header.stamp = scanTime;

		edge.header.frame_id = "vehicle";
		edge.header.stamp = scanTime;

		_pubLaserCloud.publish(velo);
		_pubCentroid.publish(cent);
		_pubEdge.publish(edge);

	}

	void SemClassifer::Clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){


		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(cloud);//Input searched pointcloud
		std::vector<pcl::PointIndices> cluster_indices; //Vector tha contains clusterized index
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;

		ece.setClusterTolerance(cluster_torelance);
		ece.setMinClusterSize(min_cluster_size);
		ece.setMaxClusterSize(cloud->points.size());

		ece.setSearchMethod(tree);
		ece.setInputCloud(cloud);

		ece.extract(cluster_indices);

		// std::cout << clustering << std::endl;

		pcl::ExtractIndices<pcl::PointXYZRGB> ei;
		ei.setInputCloud(cloud);
		ei.setNegative(false);

		for(size_t i=0; i<cluster_indices.size(); i++){
			//extract

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);

			*tmp_clustered_indices = cluster_indices[i];
			ei.setIndices(tmp_clustered_indices);
			ei.filter(*tmp_clustered_points);

			/*Input*/
			clusters.push_back(tmp_clustered_points);

		}

	}

	void SemClassifer::extract_edge_point_normal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster,const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){

		for(size_t i=0; i < cloud_normals->points.size(); i++){

			if(cloud_normals->points[i].curvature > curvaturethreshold){
				pcl::PointXYZRGB edgepoint;

				edgepoint.r = cluster->points[0].r;
				edgepoint.g = cluster->points[0].g;
				edgepoint.b = cluster->points[0].b;

				edgepoint.x = cloud_normals->points[i].normal_x;
				edgepoint.y = cloud_normals->points[i].normal_y;
				edgepoint.z = cloud_normals->points[i].normal_z;
				
				CloudEdge.push_back(edgepoint);

				std::cout << edgepoint.z << std::endl;
			}
		}
	}




	void SemClassifer::normal_edge_process(void){

		//extracting edge each cluster
		for(size_t i=0; i<clusters.size(); i++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = clusters[i];

			//Normals
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud(cluster);
			ne.setInputCloud(cluster);
			ne.setSearchMethod(tree);

			ne.setRadiusSearch(searchradius);
			
			ne.compute(*cloud_normals);

			extract_edge_point_normal(cluster, cloud_normals);
		}
	}

	void SemClassifer::extract_edge_point(const pcl::PointCloud<pcl::PointXYZRGB>& cloud){

		clusters.clear();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

		Clustering(cloudin);
		normal_edge_process();
	}


	void SemClassifer::calc_ave_point(void){

		for(size_t i=0; i<clusters.size(); i++){
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = clusters[i];

			pcl::PointXYZRGB centroid;

			float x = 0.0;
			float y = 0.0;
			float z = 0.0;

			centroid.r = cluster->points[0].r;
			centroid.g = cluster->points[0].g;
			centroid.b = cluster->points[0].b;

			for(size_t j=0; j < cluster->points.size(); j++){

				x += cluster->points[j].x;
				y += cluster->points[j].y;
				z += cluster->points[j].z;
			}
			
			//calcularate centroid
			centroid.x = x/float(cluster->points.size());
			centroid.y = y/float(cluster->points.size());
			centroid.z = z/float(cluster->points.size());

			CloudCentroid.push_back(centroid);

		}
	}

	void SemClassifer::extract_centroid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud){
		
		clusters.clear();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

		Clustering(cloudin);

		//Calculate centroid and add to CloudCentroid
		calc_ave_point();
	}

	void SemClassifer::process(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const Time& scanTime){
		size_t cloudsize = laserCloudIn.size();

		pcl::PointXYZRGB point;

		for(int i=0; i<cloudsize; i++){

			point.x = laserCloudIn[i].x;
			point.y = laserCloudIn[i].y;
			point.z = laserCloudIn[i].z;

			point.r = laserCloudIn[i].r;
			point.g = laserCloudIn[i].g;
			point.b = laserCloudIn[i].b;

			//std::cout << int(point.b) << std::endl;

			//Convert color data to 0xrrggbb
			int color_id = point.b*pow(16,0) + point.g*pow(16,2) + point.b*pow(16,4);

			if(             !pcl_isfinite(point.x) ||
					!pcl_isfinite(point.y) ||
					!pcl_isfinite(point.z)){
				continue;
			}

			if(point.x*point.x + point.y*point.y + point.z*point.z < 0.00001){
				continue;
			}

			//classify point and contain to each each semantic points
			if(!classify(point, color_id)){
				//continue;
			}

		}

		std::cout<<"a"<<std::endl;
		
		if(unlabeled.size() != 0){
			std::cout << "c" << std::endl;
			extract_centroid(unlabeled);
		}

		if(outlier.size() !=0 ){
			extract_centroid(outlier);
		}

		if(car.size() != 0 ){
			extract_centroid(car);
		}

		if(bicycle.size() != 0){
			extract_centroid(bicycle);
		}

		if(motorcycle.size() != 0 ){
			extract_centroid(motorcycle);
		}

		if(onrails.size() != 0){
			extract_centroid(onrails);
		}

		if(truck.size() != 0 ){
			extract_centroid(truck);
		}
		
		if(othervehicle.size() != 0 ){
			extract_centroid(othervehicle);
		}

		if(person.size() != 0){
			extract_centroid(othervehicle);
		}

		if(bicyclist.size() != 0){
			extract_centroid(bicyclist);
		}

		if(motorcyclist.size() != 0){
			extract_centroid(motorcyclist);
		}

		if(road.size() != 0){
			extract_edge_point(road);
		}

		if(parking.size() != 0){
			extract_edge_point(parking);
		}

		if(sidewalk.size() != 0){
			extract_edge_point(sidewalk);
		}

		if(otherground.size() != 0){
			extract_edge_point(sidewalk);
		}

		if(building.size() != 0){
			extract_edge_point(building);
		}

		if(fence.size() != 0){
			extract_centroid(fence);
		}

		if(otherstructure.size() != 0){
			extract_edge_point(otherstructure);
		}
		
		if(lanemarking.size() != 0){
			extract_centroid(lanemarking);
		}

		if(vegetation.size() != 0){
			extract_centroid(vegetation);
		}

		if(trunk.size() != 0){
			extract_centroid(trunk);
		}

		if(terrain.size() != 0){
			extract_centroid(terrain);
		}

		if(pole.size() != 0){
			extract_centroid(pole);
		}

		if(trafficsign.size() != 0){
			extract_centroid(trafficsign);
		}
		std::cout << "b" << std::endl;

		//convert pcl to ros pointcloud2 and publish pointcloud 
		publish_pointcloud(laserCloudIn, CloudCentroid, CloudEdge, scanTime);

		//clear edge and centroid
		CloudCentroid.clear();
		CloudEdge.clear();

		//clear semantic point cloud data
		unlabeled.clear();
		outlier.clear();
		car.clear();
		bicycle.clear();
		motorcycle.clear();
		onrails.clear();
		truck.clear();
		othervehicle.clear();
		person.clear();
		bicyclist.clear();
		motorcyclist.clear();
		road.clear();
		parking.clear();
		sidewalk.clear();
		otherground.clear();
		building.clear();
		fence.clear();
		otherstructure.clear();
		lanemarking.clear();
		vegetation.clear();
		trunk.clear();
		terrain.clear();
		pole.clear();
		trafficsign.clear();

	}
				
}
