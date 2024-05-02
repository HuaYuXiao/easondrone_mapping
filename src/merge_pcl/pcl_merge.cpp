//
// Created by hyx020222 on 5/2/24.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudMerger {
public:
    PointCloudMerger() : nh_("~") {
        LiDAR_sub = nh_.subscribe("/prometheus/sensors/3Dlidar_scan", 1, &PointCloudMerger::LiDAR_cb, this);
        D435i_sub = nh_.subscribe("/camera/depth/color/points", 1, &PointCloudMerger::D435i_cb, this);
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/prometheus/merged_pcl", 1);
    }
    
    void LiDAR_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        cout << "LiDAR received!" << endl;

        pcl::fromROSMsg(*msg, *LiDAR_pcl);

        mergePointClouds();
    }

    void D435i_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        cout << "D435i received!" << endl;

        pcl::fromROSMsg(*msg, *D435i_pcl);

            mergePointClouds();
    }

    void mergePointClouds() {
        cout << "Merging point clouds!" << endl;

        PointCloud merged_cloud;

        if(LiDAR_pcl){
            merged_cloud += *LiDAR_pcl;
        }
        if(D435i_pcl){
                merged_cloud += *D435i_pcl;
        }

        // Convert merged PCL PointCloud to ROS PointCloud2
        sensor_msgs::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(merged_cloud, merged_cloud_msg);
        merged_cloud_msg.header.frame_id = "map";

        merged_pub_.publish(merged_cloud_msg);

        cout << "Merged pcl published!" << endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber LiDAR_sub, D435i_sub;
    ros::Publisher merged_pub_;
    sensor_msgs::PointCloud2ConstPtr latest_LiDAR, latest_D435i;
    PointCloud::Ptr LiDAR_pcl, D435i_pcl;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    PointCloudMerger pc_merger;

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "[mapping] merge_pcl initialized!" << std::endl;

    ros::spin();
    return 0;
}
