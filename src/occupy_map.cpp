#include <occupy_map.h>


namespace octomapping{
    // 初始化函数
    void Occupy_map::init(ros::NodeHandle& nodehandle){
        // 地图原点
        nodehandle.param("map/origin_x", origin_(0), 0.0);
        nodehandle.param("map/origin_y", origin_(1), 0.0);
        nodehandle.param("map/origin_z", origin_(2), 0.0);

        // 地图实际尺寸，单位：米
        nodehandle.param("map/map_size_x", map_size_3d_(0), 6.0);
        nodehandle.param("map/map_size_y", map_size_3d_(1), 6.0);
        nodehandle.param("map/map_size_z", map_size_3d_(2), 3.0);

        min_range_ = origin_;
        max_range_ = origin_ + map_size_3d_;

        // 地图分辨率，单位：米
        nodehandle.param("map/resolution", resolution,  0.05);

        for (int i = 0; i < 3; ++i){
            // 占据图尺寸 = 地图尺寸 / 分辨率
            grid_size_(i) = ceil(map_size_3d_(i) / resolution);
        }

        // 占据容器的大小 = 占据图尺寸 x*y*z
        occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
        fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);

        // 发布点云地图
        global_pcl_pub = nodehandle.advertise<sensor_msgs::PointCloud2>("/sensor_msgs/PointCloud2",  10);
    }


    // 地图更新函数 - 输入：全局点云
    void Occupy_map::map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr & global_point){
        global_env_ptr = global_point;

         // 发布未膨胀点云
        global_pcl_pub.publish(*global_env_ptr);
    }


    // 地图更新函数 - 输入：laser
    void Occupy_map::map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom){
        // TODO 将传递过来的数据转为全局点云

        // 发布未膨胀点云
        global_pcl_pub.publish(*global_env_ptr);
    }
}
