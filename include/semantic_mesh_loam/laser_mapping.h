#ifndef __LASER_MAPPING_H
#define __LASER_MAPPING_H

#include"ros/ros.h"
#include"ros/node_handle.h"

#include"util.h"

#include"sensor_msgs/PointCloud2.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/Pose.h"

#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"

#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"pcl_conversions/pcl_conversions.h"

#include"pcl_ros/transforms.h"

#include"pcl/filters/voxel_grid.h"

namespace semloam{

	class LaserMapping{

		public:
			LaserMapping();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void odometry_callback(const nav_msgs::OdometryConstPtr& odomdata);

			void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& clouddata);

			void centroid_callback(const sensor_msgs::PointCloud2ConstPtr& centroiddata);

			void edge_callback(const sensor_msgs::PointCloud2ConstPtr& edgedata);

			void spin();

			int process(int counter);

			bool check_status();

			void status_reset();

			void get_tf_data();

			void convert_coordinate_of_pc();

			void classify_pointcloud();

			bool classify(const pcl::PointXYZRGB& point, const color_data& color_id);

			void reset_common_cloud();

			void reset_semantic_cloud();

			void do_voxel_grid();

			void voxel_grid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in, const float semantic_leafsize);

			void publish_pointcloud();

			void pointcloud_to_pcd();

		private:
			ros::Subscriber _sub_odometry;
			ros::Subscriber _sub_velodyne;
			ros::Subscriber _sub_centroid;
			ros::Subscriber _sub_edge;

			ros::Publisher _pub_pc;
			ros::Publisher _pub_odom;

			int scan_counter = 10; //Do voxel grid extraction per scan_counter
			int systemdelay = 70;

			bool pcd_save_checker = false;
			int pcd_counter = 0;

			std::string file_path = "/home/amsl/catkin_ws/PCD_file/";
			std::string file_name = "semantic_mesh_loam";

			tf::TransformListener listener;
			tf::StampedTransform map_to_laserodometry;

			pcl::PointCloud<pcl::PointXYZRGB> velo_scans;
			pcl::PointCloud<pcl::PointXYZRGB> cloud_centroid;
			pcl::PointCloud<pcl::PointXYZRGB> cloud_edge;

			const size_t velo_scan_size = 100000;
			const size_t feature_scan_size = 1000;

			Time odom_time;

			nav_msgs::Odometry odom_data;

			bool velo_checker = false;
			bool cent_checker = false;
			bool edge_checker = false;
			bool odom_checker = false;

                        pcl::PointCloud<pcl::PointXYZRGB> unlabeled;
                        pcl::PointCloud<pcl::PointXYZRGB> outlier;
                        pcl::PointCloud<pcl::PointXYZRGB> car;
                        pcl::PointCloud<pcl::PointXYZRGB> bicycle;
                        pcl::PointCloud<pcl::PointXYZRGB> bus;
                        pcl::PointCloud<pcl::PointXYZRGB> motorcycle;
                        pcl::PointCloud<pcl::PointXYZRGB> onrails;
                        pcl::PointCloud<pcl::PointXYZRGB> truck;
                        pcl::PointCloud<pcl::PointXYZRGB> othervehicle;
                        //pcl::PointCloud<pcl::PointXYZRGB> person;
                        //pcl::PointCloud<pcl::PointXYZRGB> bicyclist;
                        //pcl::PointCloud<pcl::PointXYZRGB> motorcyclist;
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

			pcl::PointCloud<pcl::PointXYZRGB> pc_map_cloud;

			size_t pc_map_cloud_size = 100*10000;

			float unlabeled_leafsize = 0.0;
			float outlier_leafsize = 0.0;
			float car_leafsize = 0.2;
			float bicycle_leafsize = 0.05;
			float bus_leafsize = 0.1;
			float motorcycle_leafsize = 0.05;
			float onrails_leafsize = 0.1;
			float truck_leafsize = 0.2;
			float othervehicle_leafsize = 0.1;
			float road_leafsize = 0.25;
			float parking_leafsize = 0.25;
			float sidewalk_leafsize = 0.25;
			float otherground_leafsize = 0.25;
			float building_leafsize = 0.25;
			float fence_leafsize = 0.25;
			float otherstructure_leafsize = 0.25;
			float lanemarking_leafsize = 0.0;
			float vegetation_leafsize = 0.15;
			float trunk_leafsize = 0.0;
			float terrain_leafsize = 0.1;
			float pole_leafsize = 0.0;
			float trafficsign_leafsize = 0.0;

			const size_t pc_size_big = 100000;
			const size_t pc_size_mid =  50000;
			const size_t pc_size_min =  10000;

	};
}

#endif
