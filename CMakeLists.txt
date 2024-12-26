cmake_minimum_required(VERSION 2.8.3)
project(easondrone_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
		tf
		pcl_ros
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
	   tf2_ros
	   tf2_sensor_msgs
	)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
       ${catkin_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(merge_pc2 
	src/merge_pc2/merge_pc2_node.cpp
	src/merge_pc2/merge_pc2.cpp
	)
target_link_libraries(merge_pc2
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${Boost_LIBRARIES}
)

catkin_install_python(PROGRAMS
		scripts/laser_to_pointcloud.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
