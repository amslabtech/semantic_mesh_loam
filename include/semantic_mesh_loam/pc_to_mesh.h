#ifndef __PC_TO_MESH_H
#define __PC_TO_MESH_H

#define PI 3.141592

#include"ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"util.h"
#include"pcl/common/common.h"
#include"pcl/features/normal_3d.h"
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/gp3.h>
#include"sensor_msgs/PointCloud2.h"
#include"pcl_conversions/pcl_conversions.h"

namespace semloam{
	
	class PcToMesh{
		
		public:
			PcToMesh();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void spin();

			void process();

			void load_PCD();

			void classify_pointcloud();

			bool classify(const pcl::PointXYZRGB& point, const color_data& color_id);

			void generate_mesh(pcl::visualization::PCLVisualizer& viewer);

			void init_config_viewer_parameter(pcl::visualization::PCLVisualizer& viewer);
			
			void generate_semantic_mesh(pcl::visualization::PCLVisualizer& viewer,const pcl::PointCloud<pcl::PointXYZRGB>& semantic_cloud, const std::string semantic_name, double search_radius, double max_neighbor);

			void config_tmp_viewer_parameter(pcl::visualization::PCLVisualizer& viewer);

			void pcl_visualize();

			void publish_rosmsg();

		private:
			std::string file_path = "/home/amsl/catkin_ws/src/semantic_mesh_loam/PCD_data/";
			std::string file_name = "semantic_mesh_loam";
			std::string file_type = "ascii"; //ascii or binary

			bool pcl_mesh_visualize_checker = true;
			bool ros_mesh_visualize_checker = false;

			int load_counter = 0;

			int v1=0;
			double normal_search_radius = 0.5;

			ros::Publisher _pubroad;
			ros::Publisher _pubcar;

			pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;

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
