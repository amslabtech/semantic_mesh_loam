add_library(semloam
	classifypointcloud.cpp
	laserodometry.cpp
	lasermapping.cpp
	pctomesh.cpp)


target_link_libraries(semloam ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
