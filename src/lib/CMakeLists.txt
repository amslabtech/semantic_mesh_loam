add_library(semloam
	classifypointcloud.cpp
	laserodometry.cpp)


target_link_libraries(semloam ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
