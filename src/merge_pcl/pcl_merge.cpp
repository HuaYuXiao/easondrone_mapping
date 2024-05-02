#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <exception>

using namespace std;

namespace message_filters {
    template <typename T>
    class Subscriber;
}

class PointCloudMerger {
public:
    PointCloudMerger();
    void LiDAR_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void D435i_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void mergePointClouds();

private:
    ros::NodeHandle nh_;
    ros::Subscriber LiDAR_sub, D435i_sub;
    ros::Publisher merged_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR_pcl, D435i_pcl;
};

PointCloudMerger::PointCloudMerger() : nh_("~") {
    LiDAR_sub = nh_.subscribe("/prometheus/sensors/3Dlidar_scan", 1, &PointCloudMerger::LiDAR_cb, this);
    D435i_sub = nh_.subscribe("/camera/depth/color/points", 1, &PointCloudMerger::D435i_cb, this);
    merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/prometheus/merged_pcl", 1);

    LiDAR_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    D435i_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "[mapping] merge_pcl initialized!" << std::endl;
}

void PointCloudMerger::LiDAR_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    try {
        cout << "LiDAR received!" << endl;

            pcl::fromROSMsg(*msg, *LiDAR_pcl);
            mergePointClouds();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in LiDAR_cb: " << e.what());
    }
}

void PointCloudMerger::D435i_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    try {
        cout << "D435i received!" << endl;

            pcl::fromROSMsg(*msg, *D435i_pcl);
            mergePointClouds();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in D435i_cb: " << e.what());
    }
}

void PointCloudMerger::mergePointClouds() {
    try {
        cout << "Merging point clouds!" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl;
        merged_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());

//        merged_pcl->points.reserve(LiDAR_pcl->size() + D435i_pcl->size());
        merged_pcl->header = LiDAR_pcl->header;

        cout << merged_pcl->header << endl;

        *merged_pcl += *LiDAR_pcl;
        *merged_pcl += *D435i_pcl;

        sensor_msgs::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(*merged_pcl, merged_cloud_msg);

        merged_cloud_msg.header.frame_id = "map";
        merged_pub_.publish(merged_cloud_msg);

        cout << "Merged pcl published!" << endl;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in mergePointClouds: " << e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    PointCloudMerger pc_merger;

    ros::spin();
    return 0;
}