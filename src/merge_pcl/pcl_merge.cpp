#include "merge_pcl/merge_pcl.h"

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
    tf::TransformListener listener_LiDAR, listener_D435i;
    ros::Publisher merged_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR_pcl, D435i_pcl;
    tf::StampedTransform transform_LiDAR, transform_D435i;
};

PointCloudMerger::PointCloudMerger() : nh_("~"){
    LiDAR_sub = nh_.subscribe("/prometheus/sensors/3Dlidar_scan", 1, &PointCloudMerger::LiDAR_cb, this);
    D435i_sub = nh_.subscribe("/camera/depth/color/points", 1, &PointCloudMerger::D435i_cb, this);
    merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/prometheus/merged_pcl", 1);

    LiDAR_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    D435i_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());

    listener_LiDAR.waitForTransform("base_link", "3Dlidar_link", ros::Time(0), ros::Duration(0.05));
    listener_D435i.waitForTransform("base_link", "D435i::camera_depth_frame", ros::Time(0), ros::Duration(0.05));

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "[mapping] merge_pcl initialized!" << std::endl;
}

void PointCloudMerger::LiDAR_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    cout << "[merge_pcl] LiDAR received!" << endl;

            pcl::fromROSMsg(*msg, *LiDAR_pcl);
    // 转换LiDAR点云到base_link坐标系
    pcl_ros::transformPointCloud("base_link", *LiDAR_pcl, *LiDAR_pcl, listener_LiDAR);

            mergePointClouds();
}

void PointCloudMerger::D435i_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        cout << "[merge_pcl] D435i received!" << endl;

            pcl::fromROSMsg(*msg, *D435i_pcl);
    // 转换D435i点云到base_link坐标系
    pcl_ros::transformPointCloud("base_link", *D435i_pcl, *D435i_pcl, listener_D435i);
    
            mergePointClouds();
}

void PointCloudMerger::mergePointClouds() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl;
        merged_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());

        merged_pcl->header.seq = LiDAR_pcl->header.seq;
        merged_pcl->header.stamp = LiDAR_pcl->header.stamp;
        merged_pcl->header.frame_id = "base_link";

        *merged_pcl += *LiDAR_pcl;
        *merged_pcl += *D435i_pcl;

        sensor_msgs::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(*merged_pcl, merged_cloud_msg);

        merged_pub_.publish(merged_cloud_msg);

        cout << "Merged pcl published!" << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    PointCloudMerger pc_merger;

    ros::spin();
    return 0;
}
