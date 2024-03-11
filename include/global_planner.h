#ifndef OCTOMAPPING
#define OCTOMAPPING


#include "occupy_map.h"


using namespace std;


namespace Global_Planning{
    extern ros::Publisher message_pub;

    class Global_Planner{
    private:
        ros::NodeHandle nodehandle;

        // 参数
        bool map_input;
        // map_groundtruth指的是真值的全局点云地图
        bool map_groundtruth;

        // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap

        // 订阅无人机状态、目标点、传感器数据（生成地图）
        ros::Subscriber drone_state_sub;

        // 支持直接输入全局已知点云
        ros::Subscriber Gpointcloud_sub;
        // 支持2维激光雷达实体传感器
        ros::Subscriber laserscan_sub;
        // ？

        // 发布控制指令
        ros::Timer mainloop_timer, track_path_timer, safety_timer;

        // 占据图类
        Occupy_map::Ptr Occupy_map_ptr;

        nav_msgs::Odometry Drone_odom;
        prometheus_msgs::DroneState _DroneState;

        // 规划器状态
        bool odom_ready;
        bool drone_ready;
        bool sensor_ready;

        // 打印的提示消息
        string message;

        // 回调函数
        void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);

        void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);

        void checkReady_cb(const ros::TimerEvent& e);


    public:
        Global_Planner(void):
            nodehandle("~")
        {}~Global_Planner(){}

        void init(ros::NodeHandle& nodehandle);
    };
}

#endif
