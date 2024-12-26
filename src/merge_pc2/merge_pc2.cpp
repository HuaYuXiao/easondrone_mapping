#include "merge_pc2/merge_pc2.h"

using namespace std;

mergePC2::mergePC2(ros::NodeHandle nh)
    : icp_utils(nh)
{
    nh.getParam("pc2_topics_in", pc2_topics_in);

    nh.param<double>("timeout", timeout, 0.5);

    hardware_concurrency = std::thread::hardware_concurrency();
    ROS_INFO("Hardware concurrency: %zu", hardware_concurrency);

    for (size_t i = 0; i < pc2_topics_in.size(); ++i) {
        string pc2_topic_in = pc2_topics_in[i];
        PointCloudT::Ptr pcT(new PointCloudT());
        pcT_buffer.push_back(pcT);

        pc2_subs.emplace_back(
            nh.subscribe<sensor_msgs::PointCloud2>(
                pc2_topic_in, hardware_concurrency, 
                boost::bind(&mergePC2::pc2Callback, this, _1, i)
            )
        );
    }

    nh.param<double>("duration", duration, 0.05);

    nh.param<string>("pc2_topic_out", pc2_topic_out, "");
    nh.param<string>("pc2_frame_out", pc2_frame_out, "");
    pc2_pub = nh.advertise<sensor_msgs::PointCloud2>(pc2_topic_out, hardware_concurrency);

    ROS_INFO("merge_pc2 node initialized success!");
}

void mergePC2::pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2_msg, const size_t index) {
    PointCloudT::Ptr pcT(new PointCloudT());
    pcl::fromROSMsg(*pc2_msg, *pcT);

    if (pcT->empty()) {
        PCL_ERROR("Empty pointcloud received! (frame_id %s)\n", pc2_msg->header.frame_id);
        return;
    }
    else{
        if (pc2_msg->header.frame_id != pc2_frame_out) {
            try {
                pcl_ros::transformPointCloud(pc2_frame_out, *pcT, *pcT, tf_listener);
            }
            catch (...) {
                ROS_ERROR("Transform failure for pointcloud! (frame_id %s)", pc2_msg->header.frame_id);
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            *pcT_buffer[index] = *pcT;
        }
    }
}

void mergePC2::mainLoop() {
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);

            if(pcT_buffer.empty()){
                PCL_ERROR("All pointclouds are empty! If this continues, be careful!\n");
            }
            else{
                PointCloudT::Ptr pcT_out(new PointCloudT());

                for (const auto& pcT : pcT_buffer) {
                    if (pcT->empty()) {
                        ROS_WARN("Empty pointcloud detected! Skipping");
                        continue;
                    }
                    else if(icp_utils.icp_enable && !pcT_out->empty()){
                        ROS_INFO("Applying ICP algorithm to pointclouds");
                        icp_utils.icpAlign(pcT_out, pcT);
                    }
                    else{
                        ROS_INFO("Simply merge pointclouds");
                        *pcT_out += *pcT;
                    }
                }

                pcl::toROSMsg(*pcT_out, pc2_out);
                pc2_out.header.frame_id = pc2_frame_out;

                pc2_pub.publish(pc2_out);
            }
        }

        // Small delay to avoid excessive CPU usage
        ros::Duration(duration).sleep();
    }
}
