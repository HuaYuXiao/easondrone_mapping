//
// Created by Eason Hua on 5/2/24.
//

#ifndef MERGE_PCL_H
#define MERGE_PCL_H

#include <thread>
#include <deque>
#include <mutex>
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
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class PointCloudMerger {
public:
    PointCloudMerger();
    ~PointCloudMerger() = default;

    // Method to process buffers
    void processBuffers();

private:
    ros::NodeHandle nh;

    int pcl2_source_num;
    size_t queue_size;
    double timeout;

    std::string pcl2_topic_0, pcl2_frame_0;
    ros::Subscriber pcl2_sub_0;
    void pcl2Callback0(const sensor_msgs::PointCloud2ConstPtr& pcl2);

    std::string pcl2_topic_1, pcl2_frame_1;
    ros::Subscriber pcl2_sub_1;
    void pcl2Callback1(const sensor_msgs::PointCloud2ConstPtr& pcl2);

    // Buffers to store point clouds
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer_0;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer_1;
    std::mutex buffer_mutex;

    // Transform listeners
    tf::TransformListener listener0, listener1;
    double tf_duration;

    std::string pcl2_topic_out, pcl2_frame_out;
    ros::Publisher pcl2_pub;
};

#endif //MERGE_PCL_H
