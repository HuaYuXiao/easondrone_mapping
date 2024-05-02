//
// Created by hyx020222 on 5/2/24.
//

#include "merge_pcl/merge_pcl.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
{
// Convert ROS PointCloud2 messages to PCL PointCloud
    PointCloud::Ptr cloud1(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);
    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);

// Merge two point clouds
    PointCloud merged_cloud;
    merged_cloud += *cloud1;
    merged_cloud += *cloud2;

    std::cout << "[mapping] pcl merged!" << std::endl;

// Convert merged PCL PointCloud to ROS PointCloud2
    sensor_msgs::PointCloud2 merged_cloud_msg;
    pcl::toROSMsg(merged_cloud, merged_cloud_msg);

// Publish the merged point cloud
    pub.publish(merged_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_pcl");
    ros::NodeHandle nh;

//    nh.param("topic1", topic1, "/prometheus/sensors/3Dlidar_scan");
//    nh.param("topic2", topic2, "/camera/depth/color/points");

// Define subscribers for the two point cloud topics
// TODO: 20hz and 16hz
    message_filters::Subscriber<sensor_msgs::PointCloud2> LiDAR_pcl_sub(nh, "/prometheus/sensors/3Dlidar_scan", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> D435i_pcl_sub(nh, "/camera/depth/color/points", 1);

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "[mapping] merge_pcl initialized!" << std::endl;

// Synchronize the two topics based on approximate time
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), LiDAR_pcl_sub, D435i_pcl_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

// Define publisher for merged point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/merged_pcl", 1);

    ros::spin();

    return 0;
}
