//
// Created by hyx020222 on 5/2/24.
//

#ifndef UAV_OCTOMAPPING_ADD_PCL_H
#define UAV_OCTOMAPPING_ADD_PCL_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub;
std::string topic1, topic2;

#endif //UAV_OCTOMAPPING_ADD_PCL_H
