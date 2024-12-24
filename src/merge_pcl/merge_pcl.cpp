#include "merge_pcl/merge_pcl.h"

using namespace std;

PointCloudMerger::PointCloudMerger() : nh("~"){
    nh.param<int>("pcl2_source_num", pcl2_source_num, 2);
    nh.param<string>("pcl2_topic_0", pcl2_topic_0, "");
    nh.param<string>("pcl2_frame_0", pcl2_frame_0, "");
    nh.param<string>("pcl2_topic_1", pcl2_topic_1, "");
    nh.param<string>("pcl2_frame_1", pcl2_frame_1, "");
    nh.param<string>("pcl2_topic_out", pcl2_topic_out, "");
    nh.param<string>("pcl2_frame_out", pcl2_frame_out, "");

    queue_size = boost::thread::hardware_concurrency();

    pcl2_sub_0.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>
    (nh, pcl2_topic_0, queue_size));
    pcl2_sub_1.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>
    (nh, pcl2_topic_1, queue_size));

    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *pcl2_sub_0, *pcl2_sub_1));
    synchronizer_->registerCallback(boost::bind(&PointCloudMerger::mergeCallback, this, _1, _2));

    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl2_topic_out, 10);

    // TODO: configurable dura
    listener0.waitForTransform(pcl2_frame_out, pcl2_frame_0, ros::Time(0), ros::Duration(0.05));
    listener1.waitForTransform(pcl2_frame_out, pcl2_frame_1, ros::Time(0), ros::Duration(0.05));

    ROS_INFO("[easondrone_mapping] merge_pcl initialized!");
}

void PointCloudMerger::mergeCallback(const sensor_msgs::PointCloud2ConstPtr& pcl2_0, const sensor_msgs::PointCloud2ConstPtr& pcl2_1) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclxyz_0, pclxyz_1, pclxyz_output;

    pclxyz_0.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl2_0, *pclxyz_0);
    // TODO: check if both frames are the same
    pcl_ros::transformPointCloud(pcl2_frame_out, *pclxyz_0, *pclxyz_0, listener1);

    pclxyz_1.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl2_1, *pclxyz_1);
    // TODO: check if both frames are the same
    pcl_ros::transformPointCloud(pcl2_frame_out, *pclxyz_1, *pclxyz_1, listener0);

    pclxyz_output.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pclxyz_output->header.seq = pclxyz_1->header.seq;
    pclxyz_output->header.stamp = pclxyz_1->header.stamp;
    pclxyz_output->header.frame_id = pcl2_frame_out;

    *pclxyz_output += *pclxyz_1;
    *pclxyz_output += *pclxyz_0;

    sensor_msgs::PointCloud2 pcl2_output;
    pcl::toROSMsg(*pclxyz_output, pcl2_output);

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
