# easondrone_mapping

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FEasonDrone_Mapping.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

A ROS package to process `PointCloud` and build `OctoMap`

- merge or concatinate multiple `PointCloud2`s, with optional ICP algorithm, and publish `PointCloud2` in another frame
- build `OctoMap` from `PointCloud2`

## Installation

```bash
sudo apt install libpcl-dev
```

```bash
cd ~/easondrone_ws/reconstruct
git clone https://github.com/HuaYuXiao/easondrone_mapping.git
cd ~/easondrone_ws
catkin_make --source reconstruct/easondrone_mapping --build reconstruct/easondrone_mapping/build
```

## 转发点云

EGO-Planner等规划器要求点云发布在world坐标系下，因此需要将原本发布在lidar_frame的点云转发到world下

```bash
rosrun easondrone_mapping pub_pcl_world.py
```

## Merge Multiple sensor_msgs::PointCloud2

### Config Parameters

```yaml
pc2_topics_in: 
    - /livox/lidar
    - /realsense/depth_camera/depth/points
timeout: 0.5

tf_duration: 0.05

# The Iterative Closest Point algorithm
icp_enable: true
icp_max_iter: 1
icp_tf_epsilon: 1e-8
icp_euclidean_fitness_epsilon: 1e-5
icp_max_corr_d: 0.05

pc2_topic_out: /sensor/pc2_out
pc2_frame_out: base_link
```

### Launch ROS Node

```xml
<launch>
    <node pkg="easondrone_mapping" type="merge_pcl" name="merge_pcl">
        <rosparam command="load" file="$(find easondrone_mapping)/config/merge_pcl.yaml" />
    </node>
</launch>
```

## Build OctoMap

### Launch ROS Node

```bash
roslaunch easondrone_mapping octomap_server.launch
```

### Save Map

> You are trying to invoke `octomap_saver` as an argument to the `octomap_server` node. However, `octomap_saver` is a node of its own, so you only have to start it from a separate terminal while `octomap_server` is running. Check the documentation at http://wiki.ros.org/octomap_server#octomap_saver

控制无人机完成建图后，用以下指令保存.ot（或者.bt，相较于.ot体积更小）格式的地图文件

```bash
rosrun octomap_server octomap_saver -f ~/easondrone_ws/reconstruct/easondrone_mapping/map.ot
```

![image](doc/log/2024-03-11/%E6%97%A0%E6%A0%87%E9%A2%98.png)

参考：
- https://octomap.github.io/octomap/doc
- ⭐️ https://wiki.ros.org/octomap
- ⭐️ https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp
- ⭐️ https://groups.google.com/g/octomap/c/ZyfNzcuGlY0?pli=1

### 查看地图

方法1：octovis

也可以借助`octovis`工具查看

```bash
octovis ~/easondrone_ws/reconstruct/easondrone_mapping/map.bt
```

方法2：rviz

一种方法是在`rviz`中查看

```bash
rviz
rosrun octomap_server octomap_server_node ~/easondrone_ws/reconstruct/easondrone_mapping/map.bt
```

添加`OccupancyGrid`，话题选择`/octomap_binary`，

![image](doc/log/2024-03-15/Snipaste_2024-03-15_14-38-27.png)

参考：
- [在ROS中将点云（PointCloud2）生成Octomap，rviz可视化显示](https://blog.csdn.net/qq_41816368/article/details/133929136)
- ⭐️ [octomap in rviz and occupancy grids in 3D maps](https://robotics.stackexchange.com/questions/41362/octomap-in-rviz-and-occupancy-grids-in-3d-maps)

### 加载地图

参考：
- ⭐ [how to use octomap_server?](https://answers.ros.org/question/361841/how-to-use-octomap_server/)
- ⭐ [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/launch/octomap_tracking_server.launch)
