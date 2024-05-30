#include "merge_pcl/merge_pcl.h"

using namespace std;

class PointCloudMerger {
public:
    PointCloudMerger();
    ~PointCloudMerger() = default;

    void mergeCallback(const sensor_msgs::PointCloud2ConstPtr& depth,
                       const sensor_msgs::PointCloud2ConstPtr& scan);

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
            SyncPolicy;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicy>> Synchronizer;

    ros::NodeHandle nh_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> LiDAR_sub_, D435i_sub_;
    Synchronizer synchronizer_;
    tf::TransformListener listener_LiDAR, listener_D435i;
    ros::Publisher merged_pub_;
    tf::StampedTransform transform_LiDAR, transform_D435i;
};

PointCloudMerger::PointCloudMerger() : nh_("~"){
    LiDAR_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/sensors/3Dlidar_scan", 50));
    D435i_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/camera/depth/color/points", 50));

    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *LiDAR_sub_, *D435i_sub_));
    synchronizer_->registerCallback(boost::bind(&PointCloudMerger::mergeCallback, this, _1, _2));

    merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensors/merged_pcl", 10);

    listener_LiDAR.waitForTransform("base_link", "3Dlidar_link", ros::Time(0), ros::Duration(0.05));
    listener_D435i.waitForTransform("base_link", "D435i::camera_depth_frame", ros::Time(0), ros::Duration(0.05));

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "[mapping] merge_pcl initialized!" << std::endl;
}

void PointCloudMerger::mergeCallback(const sensor_msgs::PointCloud2ConstPtr& depth,
                               const sensor_msgs::PointCloud2ConstPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr D435i_pcl, LiDAR_pcl, merged_pcl;

    D435i_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*depth, *D435i_pcl);
    // 转换D435i点云到map坐标系
    pcl_ros::transformPointCloud("base_link", *D435i_pcl, *D435i_pcl, listener_D435i);

    LiDAR_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*scan, *LiDAR_pcl);
    // 转换LiDAR点云到map坐标系
    pcl_ros::transformPointCloud("base_link", *LiDAR_pcl, *LiDAR_pcl, listener_LiDAR);

    merged_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());

    merged_pcl->header.seq = LiDAR_pcl->header.seq;
    merged_pcl->header.stamp = LiDAR_pcl->header.stamp;
    merged_pcl->header.frame_id = "base_link";

    *merged_pcl += *LiDAR_pcl;
    *merged_pcl += *D435i_pcl;

    sensor_msgs::PointCloud2 merged_cloud_msg;
    pcl::toROSMsg(*merged_pcl, merged_cloud_msg);

    merged_pub_.publish(merged_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    PointCloudMerger pc_merger;

    ros::spin();
    return 0;
}
