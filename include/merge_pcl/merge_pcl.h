//
// Created by Eason Hua on 5/2/24.
//

#ifndef MERGE_PCL_H
#define MERGE_PCL_H

#include <ros/ros.h>
#include <ros/rate.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <exception>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class PointCloudMerger {
public:
    PointCloudMerger();
    ~PointCloudMerger() = default;

    void mergeCallback(const sensor_msgs::PointCloud2ConstPtr& pcl2_0,
                       const sensor_msgs::PointCloud2ConstPtr& pcl2_1);

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
            SyncPolicy;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicy>> Synchronizer;

    ros::NodeHandle nh;

    int pcl2_source_num;
    std::string pcl2_topic_0, pcl2_topic_1, pcl2_topic_out;
    std::string pcl2_frame_0, pcl2_frame_1, pcl2_frame_out;

    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pcl2_sub_0, pcl2_sub_1;
    int queue_size;
    Synchronizer synchronizer_;
    tf::TransformListener listener0, listener1;
    ros::Publisher pcl2_pub;
};

#endif //MERGE_PCL_H
