add_library(semloam
	classifypointcloud.cpp)


target_link_libraries(semloam ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
