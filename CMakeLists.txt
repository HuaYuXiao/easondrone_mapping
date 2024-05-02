cmake_minimum_required(VERSION 3.0.2)
project(uav_octomapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
	   pcl_ros
       visualization_msgs
	   prometheus_msgs
		message_filters
	)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES uav_octomapping
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
	   ${PROJECT_SOURCE_DIR}/include
	   $ENV{HOME}/Prometheus/Modules/common/include
     )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(uav_octomapping
		src/octomapping_node.cpp
        src/octomapping.cpp
		src/occupy_map.cpp
		src/tools.cpp
)
target_link_libraries(uav_octomapping
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
		${OCTOMAP_LIBRARIES}
	)

add_executable(merge_pcl
		src/merge_pcl/merge_pcl.cpp
)
target_link_libraries(merge_pcl
		${catkin_LIBRARIES})

add_executable(pcl_merge
		src/merge_pcl/pcl_merge.cpp
)
target_link_libraries(pcl_merge
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)
