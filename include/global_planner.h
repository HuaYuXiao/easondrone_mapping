#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

using namespace std;

#define NODE_NAME "Global_Planner [main]"

#define MIN_DIS 0.1

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

    ros::Time tra_start_time;
    float tra_running_time;
    
    // 打印的提示消息
    string message;

    // 回调函数
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void safety_cb(const ros::TimerEvent& e);
    void checkReady_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
   

    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);

    int get_start_point_id(void);
    
public:
    Global_Planner(void):
        nodehandle("~")
    {}~Global_Planner(){}

    void init(ros::NodeHandle& nodehandle);
};

}

#endif
