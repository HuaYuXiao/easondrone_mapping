#include "merge_pcl/merge_pcl.h"

using namespace std;

PointCloudMerger::PointCloudMerger() : nh("~"){
    nh.param<int>("pcl2_source_num", pcl2_source_num, 2);
    nh.param<string>("pcl2_topic_0", pcl2_topic_0, "");
    nh.param<string>("pcl2_frame_0", pcl2_frame_0, "");
    nh.param<string>("pcl2_topic_1", pcl2_topic_1, "");
    nh.param<string>("pcl2_frame_1", pcl2_frame_1, "");
    nh.param<string>("pcl2_topic_out", pcl2_topic_out, "");

    pcl2_sub_0.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, pcl2_topic_0, 50));
    pcl2_sub_1.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, pcl2_topic_1, 50));

    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *pcl2_sub_0, *pcl2_sub_1));
    synchronizer_->registerCallback(boost::bind(&PointCloudMerger::mergeCallback, this, _1, _2));

    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl2_topic_out, 10);

    listener0.waitForTransform("base_link", pcl2_frame_0, ros::Time(0), ros::Duration(0.05));
    listener1.waitForTransform("base_link", pcl2_frame_1, ros::Time(0), ros::Duration(0.05));

    std::cout << "[easondrone_mapping] merge_pcl initialized!" << std::endl;
}

void PointCloudMerger::mergeCallback(const sensor_msgs::PointCloud2ConstPtr& pcl2_0, const sensor_msgs::PointCloud2ConstPtr& pcl2_1) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz_0, pcl_xyz_1, pcl_xyz_output;

    pcl_xyz_0.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl2_0, *pcl_xyz_0);
    // 转换D435i点云到map坐标系
    pcl_ros::transformPointCloud("base_link", *pcl_xyz_0, *pcl_xyz_0, listener1);

    pcl_xyz_1.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl2_1, *pcl_xyz_1);
    // 转换LiDAR点云到map坐标系
    pcl_ros::transformPointCloud("base_link", *pcl_xyz_1, *pcl_xyz_1, listener0);

    pcl_xyz_output.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl_xyz_output->header.seq = pcl_xyz_1->header.seq;
    pcl_xyz_output->header.stamp = pcl_xyz_1->header.stamp;
    pcl_xyz_output->header.frame_id = "base_link";

    *pcl_xyz_output += *pcl_xyz_1;
    *pcl_xyz_output += *pcl_xyz_0;

    sensor_msgs::PointCloud2 pcl2_output;
    pcl::toROSMsg(*pcl_xyz_output, pcl2_output);

    if(pcl2_pub){
        pcl2_pub.publish(pcl2_output);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pcl");
    PointCloudMerger pc_merger;

    ros::spin();
    return 0;
}
