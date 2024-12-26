//
// Created by Eason Hua on 5/2/24.
//

#ifndef MERGE_PCL_H
#define MERGE_PCL_H

#include <thread>
#include <deque>
#include <mutex>
#include <vector>
#include <exception>

#include <ros/ros.h>
#include <ros/rate.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

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
    // Subscribers
    std::vector<std::string> pc2_topics_in;
    size_t hardware_concurrency;
    std::vector<ros::Subscriber> pc2_subs;

    double timeout;

    // Transform listener
    double duration;
    tf::TransformListener tf_listener;

    // The Iterative Closest Point algorithm
    bool icp_enable;
    int icp_max_iter;
    double icp_tf_epsilon;
    double icp_euclidean_fit_epsilo;
    double icp_max_coor_d;

    // Buffers to store point clouds
    std::deque<PointCloudT::Ptr> pcT_buffer;
    std::mutex buffer_mutex;

    // Publisher
    std::string pc2_topic_out;
    std::string pc2_frame_out;
    ros::Publisher pc2_pub;

    // Unified callback for point cloud subscribers
    void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2, const size_t index);
    void icpAlgorithm(PointCloudT::Ptr& pcT_out, const PointCloudT::Ptr& pcT);
};

#endif //MERGE_PCL_H
