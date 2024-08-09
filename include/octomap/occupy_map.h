#ifndef _OCCUPY_MAP_H
#define _OCCUPY_MAP_H

#include "tools.h"

namespace octomapping{
    class Occupy_map{
        public:
            Occupy_map(){}

            // 定义该类的指针
            typedef std::shared_ptr<Occupy_map> Ptr;

            // 地图原点,地图尺寸
            Eigen::Vector3d origin_, map_size_3d_;
            // 地图分辨率
            double resolution;

            // 发布点云用于rviz显示
            ros::Publisher global_pcl_pub;

            // 全局点云指针
            sensor_msgs::PointCloud2ConstPtr global_env_ptr;

            //初始化
            void init(ros::NodeHandle& nodehandle);

            // 地图更新函数 - 输入：全局点云
            void map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr &global_point);

            // 地图更新函数 - 输入：二维激光雷达
            void map_update_laser(const sensor_msgs::LaserScanConstPtr &local_point, const nav_msgs::Odometry &odom);
    };
}
#endif
