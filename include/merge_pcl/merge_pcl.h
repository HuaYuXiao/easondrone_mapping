//
// Created by Eason Hua on 5/2/24.
//

#ifndef MERGE_PCL_H
#define MERGE_PCL_H

#include <thread>
#include <deque>
#include <mutex>
#include <vector>
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

    // Parameters
    std::vector<std::string> pcl2_topics_in;
    size_t queue_size;
    std::vector<ros::Subscriber> pcl2_subs;
    double timeout;

    // Transform listeners
    std::vector<std::shared_ptr<tf::TransformListener>> tf_listeners;
    double tf_duration;

    // Buffers to store point clouds
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclxyz_buffer;
    std::mutex buffer_mutex;

    // Output topic and frame
    std::string pcl2_topic_out;
    std::string pcl2_frame_out;
    ros::Publisher pcl2_pub;

    // Unified callback for point cloud subscribers
    void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2, size_t index);
};

#endif //MERGE_PCL_H
