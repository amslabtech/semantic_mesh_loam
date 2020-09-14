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
		
		cluster_torelance = 0.5; //When point distance is over 0.5, consider it other cluster
		min_cluster_size = 40;

		clusters.reserve(cluster_size);
		empty_vec.reserve(clusters.size());

		unlabeled.reserve(pc_size_min);
		outlier.reserve(pc_size_min);
		car.reserve(pc_size_mid);
		bicycle.reserve(pc_size_min);
		motorcycle.reserve(pc_size_min);
		onrails.reserve(pc_size_min);
		truck.reserve(pc_size_mid);
		othervehicle.reserve(pc_size_mid);
		person.reserve(pc_size_min);
		bicyclist.reserve(pc_size_min);
		motorcyclist.reserve(pc_size_min);
		road.reserve(pc_size_big);
		parking.reserve(pc_size_mid);
		sidewalk.reserve(pc_size_mid);
		otherground.reserve(pc_size_mid);
		building.reserve(pc_size_big);
		fence.reserve(pc_size_big);
		otherstructure.reserve(pc_size_min);
		lanemarking.reserve(pc_size_min);
		vegetation.reserve(pc_size_mid);
		trunk.reserve(pc_size_min);
		terrain.reserve(pc_size_mid);;
		pole.reserve(pc_size_min);
		trafficsign.reserve(pc_size_min);

		CloudCentroid.reserve(pc_size_min);
		CloudEdge.reserve(pc_size_min);


		searchradius = 0.2;//Radius used to calculate point normal tekitou
		curvaturethreshold = 0.15; //tekitou 
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

		//std::cout << "laser" << std::endl;
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

	bool SemClassifer::classify(const pcl::PointXYZRGB& point, const color_data& color_id){

		if(color_id.r==0 && color_id.g==0 && color_id.b==0){
			unlabeled.push_back(point);
			//std::cout << "unlabeled" << std::endl;
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
			outlier.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==150 && color_id.b==245){
			car.push_back(point);
			//std::cout << "car" << std::endl;
		}
		else if(color_id.r==100 && color_id.g==230 && color_id.b==245){
			bicycle.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==80 && color_id.b==250){
			bus.push_back(point);
		}
		else if(color_id.r==30 && color_id.g==60 && color_id.b==150){
			motorcycle.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==0 && color_id.b==255){
			onrails.push_back(point);
		}
		else if(color_id.r==80 && color_id.g==30 && color_id.b==180){
			truck.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==30 && color_id.b==30){
			person.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==40 && color_id.b==200){
			bicyclist.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==30 && color_id.b==90){
			motorcyclist.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==255){
			road.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==255){
			parking.push_back(point);
		}
		else if(color_id.r==75 && color_id.g==0 && color_id.b==75){
			sidewalk.push_back(point);
		}
		else if(color_id.r==175 && color_id.g==0 && color_id.b==75){
			otherground.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==200 && color_id.b==0){
			building.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==120 && color_id.b==50){
			fence.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==0){
			otherstructure.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==255 && color_id.b==170){
			lanemarking.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==175 && color_id.g==0){
			vegetation.push_back(point);
		}
		else if(color_id.r==135 && color_id.g==60 && color_id.b==0){
			trunk.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==240 && color_id.b==80){
			terrain.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==240 && color_id.b==150){
			pole.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
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

		velo.header.frame_id = "velodyne";
		velo.header.stamp = scanTime;

		cent.header.frame_id = "velodyne";
		cent.header.stamp = scanTime;

		edge.header.frame_id = "velodyne";
		edge.header.stamp = scanTime;

		_pubLaserCloud.publish(velo);
		_pubCentroid.publish(cent);
		_pubEdge.publish(edge);

	}

	int SemClassifer::Clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){


		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(cloud);//Input searched pointcloud
		//std::cout << "set cloud kdtree" <<std::endl;
		std::vector<pcl::PointIndices> cluster_indices; //Vector tha contains clusterized index
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;

		ece.setClusterTolerance(cluster_torelance);
		ece.setMinClusterSize(min_cluster_size);
		ece.setMaxClusterSize(cloud->points.size());

		ece.setSearchMethod(tree);
		ece.setInputCloud(cloud);

		//std::cout << "set ece" << std::endl;

		ece.extract(cluster_indices);

		//std::cout << "clustering" << std::endl;

		pcl::ExtractIndices<pcl::PointXYZRGB> ei;
		ei.setInputCloud(cloud);
		ei.setNegative(false);

		//std::cout << "ei.set" << std::endl;

		int ac_number = 0;

		//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

		for(size_t i=0; i<cluster_indices.size(); i++){
			//extract

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);

			*tmp_clustered_indices = cluster_indices[i];
			ei.setIndices(tmp_clustered_indices);
			ei.filter(*tmp_clustered_points);

			/*Input*/
			clusters.push_back(tmp_clustered_points);
			//clusters[ac_number] = tmp_clustered_points;
			ac_number += 1;
			/*
			std::cout << "tmp clear " << std::endl;
			tmp_clustered_points->points.clear();
			std::cout << "clear end" << std::endl;
			*/

		}

		//return ac_number;
		return ac_number;

	}

	void SemClassifer::extract_edge_point_normal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster,const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){

		for(size_t i=0; i < cloud_normals->points.size(); i++){

			if(cloud_normals->points[i].curvature > curvaturethreshold){
				pcl::PointXYZRGB edgepoint;

				edgepoint.r = cluster->points[0].r;
				edgepoint.g = cluster->points[0].g;
				edgepoint.b = cluster->points[0].b;

				/*
				edgepoint.x = cloud_normals->points[i].normal_x;
				edgepoint.y = cloud_normals->points[i].normal_y;
				edgepoint.z = cloud_normals->points[i].normal_z;
				*/
				
				edgepoint.x = cluster->points[i].x;
				edgepoint.y = cluster->points[i].y;
				edgepoint.z = cluster->points[i].z;
			
				CloudEdge.push_back(edgepoint);

				//std::cout << edgepoint.z << std::endl;
			}
		}
	}




	void SemClassifer::normal_edge_process(int cluster_num){

		//extracting edge each cluster
		for(int i=0; i<cluster_num; i++){
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
		//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

		int cluster_num = Clustering(cloudin);
		if(cluster_num > 0){
			normal_edge_process(cluster_num);
			//std::cout << clusters.size() << std::endl;
		}

		clusters = empty_vec;
	}


	void SemClassifer::calc_ave_point(int cluster_num){

		//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator itr;

		for(int i=0; i < cluster_num; i++){

			//std::cout << "avasgda" << std::endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = clusters[i];

			pcl::PointXYZRGB centroid;

			float x = 0.0;
			float y = 0.0;
			float z = 0.0;

			centroid.r = cluster->points[0].r;
			centroid.g = cluster->points[0].g;
			centroid.b = cluster->points[0].b;

			//std::cout << clusters.size() << std::endl;

			for(size_t j=0; j < cluster->points.size(); j++){

				x += cluster->points[j].x;
				y += cluster->points[j].y;
				z += cluster->points[j].z;
			}

			//std::cout << "aaa" << std::endl;
			
			//calcularate centroid
			centroid.x = x/float(cluster->points.size());
			centroid.y = y/float(cluster->points.size());
			centroid.z = z/float(cluster->points.size());

			//std::cout << centroid.x << std::endl;

			//std::cout << "calc cent" << std::endl;
			CloudCentroid.push_back(centroid);
			//std::cout << "push cent" << std::endl;


		}
	}

	void SemClassifer::extract_centroid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

		//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

		int cluster_num = Clustering(cloudin);

		//Calculate centroid and add to CloudCentroid
		if(cluster_num > 0){
			calc_ave_point(cluster_num);
			//std::cout <<clusters.size() << std::endl;
		}

		clusters = empty_vec;
		
	}

	void SemClassifer::process(const pcl::PointCloud<pcl::PointXYZRGB>& laserCloudIn, const Time& scanTime){
		size_t cloudsize = laserCloudIn.size();
		//std::cout << cloudsize << std::endl
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
			//int color_id = point.b*pow(16,0) + point.g*pow(16,2) + point.b*pow(16,4);
			color_data color_id;
			color_id.r = int(point.r);
			color_id.g = int(point.g);
			color_id.b = int(point.b);

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

		//std::cout<<"a"<<std::endl;
		
		if(unlabeled.size() != 0){
			//std::cout << "unlabeled" << unlabeled.size() <<std::endl;
			//extract_centroid(unlabeled);
		}

		if(outlier.size() !=0 ){
			//std::cout <<"outlier" << std::endl;
			//extract_centroid(outlier);
		}

		if(car.size() != 0 ){
			//std::cout << "car" << std::endl;
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
			extract_centroid(person);
		}

		if(bicyclist.size() != 0){
			extract_centroid(bicyclist);
		}

		if(motorcyclist.size() != 0){
			extract_centroid(motorcyclist);
		}

		if(road.size() != 0){
			//std::cout << "road" << std::endl;
			//extract_edge_point(road);
		}

		if(parking.size() != 0){
			//extract_edge_point(parking);
		}

		if(sidewalk.size() != 0){
			//extract_edge_point(sidewalk);
		}

		if(otherground.size() != 0){
			//extract_edge_point(otherground);
		}

		if(building.size() != 0){
			extract_edge_point(building);
		}

		if(fence.size() != 0){
			extract_edge_point(fence);
		}

		if(otherstructure.size() != 0){
			//extract_edge_point(otherstructure);
		}
		
		if(lanemarking.size() != 0){
			extract_centroid(lanemarking);
		}

		if(vegetation.size() != 0){
			extract_centroid(vegetation);
		}

		if(trunk.size() != 0){
			//extract_centroid(trunk);
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
		//std::cout << "b" << std::endl;

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
