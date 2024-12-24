#include "merge_pcl/merge_pcl.h"

using namespace std;

PointCloudMerger::PointCloudMerger() : nh("~") {
    nh.param<int>("pcl2_source_num", pcl2_source_num, 2);
    queue_size = boost::thread::hardware_concurrency();

    nh.param<string>("pcl2_topic_0", pcl2_topic_0, "");
    nh.param<string>("pcl2_frame_0", pcl2_frame_0, "");
    pcl2_sub_0 = nh.subscribe(pcl2_topic_0, queue_size, &PointCloudMerger::pcl2Callback0, this);

    nh.param<string>("pcl2_topic_1", pcl2_topic_1, "");
    nh.param<string>("pcl2_frame_1", pcl2_frame_1, "");
    pcl2_sub_1 = nh.subscribe(pcl2_topic_1, queue_size, &PointCloudMerger::pcl2Callback1, this);

    nh.param<double>("tf_duration", tf_duration, 0.1);
    listener0.waitForTransform(pcl2_frame_out, pcl2_frame_0, ros::Time(0), ros::Duration(tf_duration));
    listener1.waitForTransform(pcl2_frame_out, pcl2_frame_1, ros::Time(0), ros::Duration(tf_duration));

    nh.param<string>("pcl2_topic_out", pcl2_topic_out, "");
    nh.param<string>("pcl2_frame_out", pcl2_frame_out, "");
    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl2_topic_out, queue_size);

    ROS_INFO("[merge_pcl] Initialized!");
}

void PointCloudMerger::pcl2Callback0(const sensor_msgs::PointCloud2ConstPtr& pcl2) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffer_0.push_back(pcl2);
    if (buffer_0.size() > queue_size) {
        buffer_0.pop_front();
    }
}

void PointCloudMerger::pcl2Callback1(const sensor_msgs::PointCloud2ConstPtr& pcl2) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffer_1.push_back(pcl2);
    if (buffer_1.size() > queue_size) {
        buffer_1.pop_front();
    }
}

void PointCloudMerger::processBuffers() {
    while (ros::ok()) {
        sensor_msgs::PointCloud2ConstPtr pcl2_0 = nullptr;
        sensor_msgs::PointCloud2ConstPtr pcl2_1 = nullptr;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            if (!buffer_0.empty() && !buffer_1.empty()) {
                pcl2_0 = buffer_0.front();
                pcl2_1 = buffer_1.front();
                buffer_0.pop_front();
                buffer_1.pop_front();
            }
        }

        if (pcl2_0 && pcl2_1) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclxyz_0, pclxyz_1, pclxyz_output;

            pclxyz_0.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pcl2_0, *pclxyz_0);
            if (pcl2_0->header.frame_id != pcl2_frame_out) {
                try {
                    pcl_ros::transformPointCloud(pcl2_frame_out, *pclxyz_0, *pclxyz_0, listener0);
                } catch (...) {
                    ROS_ERROR("Transform failed for pcl2_0!");
                    continue;
                }
            }

            pclxyz_1.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pcl2_1, *pclxyz_1);
            if (pcl2_1->header.frame_id != pcl2_frame_out) {
                try {
                    pcl_ros::transformPointCloud(pcl2_frame_out, *pclxyz_1, *pclxyz_1, listener1);
                } catch (...) {
                    ROS_ERROR("Transform failed for pcl2_1!");
                    continue;
                }
            }

            pclxyz_output.reset(new pcl::PointCloud<pcl::PointXYZ>());
            *pclxyz_output += *pclxyz_0;
            *pclxyz_output += *pclxyz_1;

            sensor_msgs::PointCloud2 pcl2_output;
            pcl::toROSMsg(*pclxyz_output, pcl2_output);

            pcl2_output.header.frame_id = pcl2_frame_out;
            pcl2_pub.publish(pcl2_output);
        }

        ros::Duration(0.01).sleep();  // Small delay to avoid excessive CPU usage
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pcl");
    PointCloudMerger pc_merger;

    boost::thread processing_thread(&PointCloudMerger::processBuffers, &pc_merger);

    ros::spin();
    processing_thread.join();

    return 0;
}
