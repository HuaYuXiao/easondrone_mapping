#include "merge_pcl/merge_pcl.h"

using namespace std;

PointCloudMerger::PointCloudMerger(ros::NodeHandle nh){
    nh.getParam("pc2_topics_in", pc2_topics_in);
    nh.param<double>("timeout", timeout, 0.5);

    queue_size = std::thread::hardware_concurrency();

    for (size_t i = 0; i < pc2_topics_in.size(); ++i) {
        string pc2_topic_in = pc2_topics_in[i];
        PointCloudT::Ptr pcT(new PointCloudT());
        pcT_buffer.push_back(pcT);

        pc2_subs.emplace_back(
            nh.subscribe<sensor_msgs::PointCloud2>(
                pc2_topic_in, queue_size, 
                boost::bind(&PointCloudMerger::pc2Callback, this, _1, i)
            )
        );
    }

    nh.param<double>("tf_duration", tf_duration, 0.05);

    nh.param<string>("pc2_topic_out", pc2_topic_out, "");
    nh.param<string>("pc2_frame_out", pc2_frame_out, "");
    pc2_pub = nh.advertise<sensor_msgs::PointCloud2>(pc2_topic_out, queue_size);

    ROS_INFO("merge_pcl node initialized success!");
}

void PointCloudMerger::pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2_msg, const size_t index) {
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

void PointCloudMerger::processBuffers() {
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
                        continue;
                    }
                    else if(pcT_out->empty()){
                        *pcT_out += *pcT;
                    }
                    else{
                        // TODO: Optional ICP
                        *pcT_out += *pcT;
                    }
                }

                sensor_msgs::PointCloud2 pc2_out;
                pcl::toROSMsg(*pcT_out, pc2_out);
                pc2_out.header.frame_id = pc2_frame_out;

                pc2_pub.publish(pc2_out);
            }
        }

        // Small delay to avoid excessive CPU usage
        ros::Duration(tf_duration).sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pcl");
    ros::NodeHandle nh("~");
    
    PointCloudMerger merge_pcl(nh);

    std::thread process_thread(&PointCloudMerger::processBuffers, &merge_pcl);

    ros::spin();

    return 0;
}
