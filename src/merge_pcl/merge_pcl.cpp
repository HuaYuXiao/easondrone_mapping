#include "merge_pcl/merge_pcl.h"

using namespace std;

PointCloudMerger::PointCloudMerger() : nh("~") {
    nh.getParam("pcl2_topics_in", pcl2_topics_in);
    nh.param<double>("timeout", timeout, 0.5);

    queue_size = boost::thread::hardware_concurrency();

    for (size_t i = 0; i < pcl2_topics_in.size(); ++i) {
        string pcl2_topic_in = pcl2_topics_in[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclxyz(new pcl::PointCloud<pcl::PointXYZ>());
        pclxyz_buffer.push_back(pclxyz);
        tf_listeners.emplace_back(new tf::TransformListener());

        pcl2_subs.emplace_back(
            nh.subscribe<sensor_msgs::PointCloud2>(
                pcl2_topic_in, queue_size, 
                boost::bind(&PointCloudMerger::pcl2Callback, this, _1, i)
            )
        );
    }

    nh.param<double>("tf_duration", tf_duration, 0.05);

    nh.param<string>("pcl2_topic_out", pcl2_topic_out, "");
    nh.param<string>("pcl2_frame_out", pcl2_frame_out, "");
    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl2_topic_out, queue_size);

    ROS_INFO("[merge_pcl] Initialized success!");
}

void PointCloudMerger::pcl2Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg, size_t index) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclxyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl2_msg, *pclxyz);

    if (pcl2_msg->header.frame_id != pcl2_frame_out) {
        try {
            pcl_ros::transformPointCloud(pcl2_frame_out, *pclxyz, *pclxyz, *tf_listeners[index]);
        } 
        catch (...) {
            ROS_ERROR("Transform failed for pcl2_topic_in index %zu!", index);
            return;
        }
    }

    std::lock_guard<std::mutex> lock(buffer_mutex);
    *pclxyz_buffer[index] = *pclxyz;
}

void PointCloudMerger::processBuffers() {
    while (ros::ok()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclxyz_out(new pcl::PointCloud<pcl::PointXYZ>());

        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            for (const auto& pclxyz : pclxyz_buffer) {
                if (!pclxyz->empty()) {
                    *pclxyz_out += *pclxyz;
                }
            }
        }

        if (!pclxyz_out->empty()) {
            sensor_msgs::PointCloud2 pcl2_out;
            pcl::toROSMsg(*pclxyz_out, pcl2_out);
            pcl2_out.header.frame_id = pcl2_frame_out;

            pcl2_pub.publish(pcl2_out);
        }
        else{
            ROS_WARN("empty pointcloud!");
        }

        ros::Duration(tf_duration).sleep();  // Small delay to avoid excessive CPU usage
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pcl");
    PointCloudMerger merge_pcl;

    boost::thread process_thread(&PointCloudMerger::processBuffers, &merge_pcl);

    ros::spin();
    process_thread.join();

    return 0;
}
