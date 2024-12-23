cmake_minimum_required(VERSION 3.0.2)
project(easondrone_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
		message_filters
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
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(merge_pcl src/merge_pcl/merge_pcl.cpp)
target_link_libraries(merge_pcl
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS
		scripts/laser_to_pointcloud.py
		scripts/pub_pcl_world.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
