cmake_minimum_required(VERSION 3.0.2)
project(semantic_mesh_loam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions
	DEPENDS EIGEN3 PCL
	INCLUDE_DIRS include
	LIBRARIES semloam
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src/lib)

add_executable(classify_pointcloud src/classify_pointcloud.cpp)
target_link_libraries(classify_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} semloam)

add_executable(laser_odometry src/laser_odometry.cpp)
target_link_libraries(laser_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} semloam)



#endif()




