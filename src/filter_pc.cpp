#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub_filtered;

void cloudCallback(const sensor_msgs::PointCloudConstPtr& pc) {
    // Define the radius threshold
    const double radius = 0.4;

    // Convert sensor_msgs::PointCloud to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcT(new pcl::PointCloud<pcl::PointXYZ>());

    // Loop through the points and filter based on the radius condition
    for (const auto& point : pc->points) {
        if (abs(point.x) > radius || abs(point.y) > radius || abs(point.z) > radius){
            pcT->points.emplace_back(point.x, point.y, point.z);  // Keep points beyond the radius
        }
    }

    // Set metadata for the filtered pcT (points outside the radius)
    pcT->width = pcT->points.size();
    pcT->height = 1;
    pcT->is_dense = false;

    // Convert the filtered pcT to ROS PointCloud2
    sensor_msgs::PointCloud2 pc2_filtered;
    pcl::toROSMsg(*pcT, pc2_filtered);
    pc2_filtered.header = pc->header;  // Preserve header from the original message

    // Publish the filtered and removed PointCloud2 messages
    pub_filtered.publish(pc2_filtered);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_pc");
    ros::NodeHandle nh;

    // Subscribe to the input sensor_msgs::PointCloud topic
    ros::Subscriber sub = nh.subscribe("/livox/lidar_raw", 10, cloudCallback);

    // Advertise the output sensor_msgs::PointCloud2 topic for filtered points
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 10);

    ros::spin();
}
