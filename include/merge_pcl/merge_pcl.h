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
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

class PointCloudMerger {
public:
    PointCloudMerger(ros::NodeHandle nh);
    ~PointCloudMerger() = default;

    // Method to process buffers
    void processBuffers();

private:
    // Parameters
    std::vector<std::string> pc2_topics_in;
    size_t queue_size;
    std::vector<ros::Subscriber> pc2_subs;
    double timeout;

    // Transform listener
    double tf_duration;
    tf::TransformListener tf_listener;

    // Buffers to store point clouds
    std::deque<PointCloudT::Ptr> pcT_buffer;
    std::mutex buffer_mutex;

    // Output topic and frame
    std::string pc2_topic_out;
    std::string pc2_frame_out;
    ros::Publisher pc2_pub;

    // Unified callback for point cloud subscribers
    void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2, size_t index);
};

#endif //MERGE_PCL_H
