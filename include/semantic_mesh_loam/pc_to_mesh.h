#ifndef __PC_TO_MESH_H
#define __PC_TO_MESH_H

#include"ros/ros.h"

namespace semloam{
	
	class PcToMesh{
		
		public:
			PcToMesh();

			bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

			void spin();

		private:
			std::string file_path = "/home/amsl/catkin_ws/src/semantic_mesh_loam/PCD_file/";
			std::string file_name = "semantic_mesh_loam";
			std::string file_type = "ascii"; //ascii or binary

			bool pcl_mesh_visualize_checker = true;
			bool ros_mesh_visualize_checker = false;
	};
}

#endif
