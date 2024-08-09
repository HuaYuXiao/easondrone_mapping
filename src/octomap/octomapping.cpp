#include "octomap/octomapping.h"

namespace octomapping{
    // 初始化函数
    void OctoMapping::init(ros::NodeHandle& nodehandle){
        // 选择地图更新方式：　true代表全局点云，false代表激光雷达scan数据
        nodehandle.param("global_planner/map_input", map_input, true);

        // 根据map_input选择地图更新方式
        if(map_input){
            Gpointcloud_sub = nodehandle.subscribe<sensor_msgs::PointCloud2>("/sensor_msgs/PointCloud2", 1, &OctoMapping::Gpointcloud_cb, this);
        }
        else{
            laserscan_sub = nodehandle.subscribe<sensor_msgs::LaserScan>("/easondrone/global_planning/laser_scan", 1, &OctoMapping::laser_cb, this);
        }

        // 初始化占据地图
        Occupy_map_ptr.reset(new Occupy_map);
        Occupy_map_ptr->init(nodehandle);
    }

    // 根据全局点云更新地图：已知全局点云的场景、由SLAM实时获取的全局点云
    void OctoMapping::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
        // 此时的点云要么是仿真下的点云插件生成的点云或者是实机中传感器通过octomap生成的点云，这种就需要循环10次进行膨胀层
        static int update_num=0;
        update_num++;

        // TODO: 此处改为根据循环时间计算的数值
        if(update_num == 10){
            // 对地图进行更新
            Occupy_map_ptr->map_update_gpcl(msg);
            update_num = 0;
        }
    }

    // 根据2维雷达数据更新地图：2维激光雷达
    void OctoMapping::laser_cb(const sensor_msgs::LaserScanConstPtr &msg){
        /* need odom_ for center radius sensing */

        // 对地图进行更新（laser+odom）
        Occupy_map_ptr->map_update_laser(msg, Drone_odom);
    }
}
