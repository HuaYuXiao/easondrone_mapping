#include "global_planner.h"


namespace Global_Planning{
    // 初始化函数
    void Global_Planner::init(ros::NodeHandle& nodehandle){
        // 选择地图更新方式：　true代表全局点云，false代表激光雷达scan数据
        nodehandle.param("global_planner/map_input", map_input, true);
        nodehandle.param("global_planner/map_groundtruth", map_groundtruth, false); 


        // 订阅 无人机状态
        drone_state_sub = nodehandle.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Global_Planner::drone_state_cb, this);

        // 根据map_input选择地图更新方式
        if(map_input){
            Gpointcloud_sub = nodehandle.subscribe<sensor_msgs::PointCloud2>("/sensor_msgs/PointCloud2", 1, &Global_Planner::Gpointcloud_cb, this);
        }else{
            laserscan_sub = nodehandle.subscribe<sensor_msgs::LaserScan>("/prometheus/global_planning/laser_scan", 1, &Global_Planner::laser_cb, this);
        }


        // 发布提示消息
        message_pub = nodehandle.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);


        // 定时器 规划器算法执行周期
        mainloop_timer = nodehandle.createTimer(ros::Duration(1), &Global_Planner::checkReady_cb, this);


        // 初始化占据地图
        Occupy_map_ptr.reset(new Occupy_map);
        Occupy_map_ptr->init(nodehandle);


        // 规划器状态参数初始化
        odom_ready = false;
        drone_ready = false;
        sensor_ready = false;
    }


    void Global_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg){
        _DroneState = *msg;

        odom_ready = true;

        if (_DroneState.connected == true && _DroneState.armed == true ){
            drone_ready = true;
        }else{
            drone_ready = false;
        }

        // odem is needed only when using laser scan
        if(!map_input) {
            Drone_odom.header = _DroneState.header;
            Drone_odom.child_frame_id = "base_link";

            Drone_odom.pose.pose.position.x = _DroneState.position[0];
            Drone_odom.pose.pose.position.y = _DroneState.position[1];
            Drone_odom.pose.pose.position.z = _DroneState.position[2];

            // TODO 这里需要做四元数转换吗？
            Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
            Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
            Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
            Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
        }
    }


    // 根据全局点云更新地图
    // 情况：已知全局点云的场景、由SLAM实时获取的全局点云
    void Global_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
        /* need odom_ for center radius sensing */
        if (!odom_ready){
            return;
        }

        sensor_ready = true;

        // 如果为true，那就是使用虚拟的点云数据，所以就不用循环
        if(!map_groundtruth){
            // 对地图进行更新
            Occupy_map_ptr->map_update_gpcl(msg);
        }else{
            // 默认情况下为false，此时的点云要么是仿真下的点云插件生成的点云或者是实机中传感器通过octomap生成的点云，这种就需要循环10次进行膨胀层
            static int update_num=0;
            update_num++;

            // 此处改为根据循环时间计算的数值
            if(update_num == 10){
                // 对地图进行更新
                Occupy_map_ptr->map_update_gpcl(msg);
                update_num = 0;
            }
        }
    }


    // 根据2维雷达数据更新地图
    // 情况：2维激光雷达
    void Global_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg){
        /* need odom_ for center radius sensing */
        if (!odom_ready){
            return;
        }

        sensor_ready = true;

        // 对地图进行更新（laser+odom）
        Occupy_map_ptr->map_update_laser(msg, Drone_odom);
    }


    // 主循环 
    void Global_Planner::checkReady_cb(const ros::TimerEvent& e){
        message = "";
        // 检查当前状态，不满足规划条件则直接退出主循环
        // TODO 此处打印消息与后面的冲突了，逻辑上存在问题
        if(!odom_ready || !drone_ready || !sensor_ready){
            if(!odom_ready){
                message += "Need odom info! ";
            }else if(!drone_ready){
                message += "Drone is not ready! ";
            }else if(!sensor_ready){
                message += "Need sensor info! ";
            }
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
        }
        return;
    }
}
