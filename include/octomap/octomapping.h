#ifndef OCTOMAPPING
#define OCTOMAPPING

#include "occupy_map.h"

using namespace std;

namespace octomapping{
    class OctoMapping{
    private:
        ros::NodeHandle nodehandle;

        // 参数
        bool map_input;

        // 订阅无人机状态、目标点、传感器数据（生成地图）
        ros::Subscriber drone_state_sub;
        // 支持直接输入全局已知点云
        ros::Subscriber Gpointcloud_sub;
        // 支持2维激光雷达实体传感器
        ros::Subscriber laserscan_sub;

        // 占据图类
        Occupy_map::Ptr Occupy_map_ptr;

        nav_msgs::Odometry Drone_odom;

        // 回调函数
        void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);

    public:
        OctoMapping(void):
            nodehandle("~")
        {}~OctoMapping(){}

        void init(ros::NodeHandle& nodehandle);
    };
}

#endif
