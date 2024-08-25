# easondrone_mapping

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FEasonDrone_Mapping.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)

A ROS package for mapping via octomap.

## Installation

```bash
cd ~/EasonDrone
catkin_make --source Reconstruction/easondrone_mapping --build Reconstruction/easondrone_mapping/build
```

## 转发点云

EGO-Planner等规划器要求点云发布在world坐标系下，因此需要将原本发布在lidar_frame的点云转发到world下

```bash
rosrun easondrone_mapping pub_pcl_world.py
```


## 建立地图

```bash
roslaunch easondrone_mapping simulation.launch
```

## 保存地图

> You are trying to invoke `octomap_saver` as an argument to the `octomap_server` node. However, `octomap_saver` is a node of its own, so you only have to start it from a separate terminal while `octomap_server` is running. Check the documentation at http://wiki.ros.org/octomap_server#octomap_saver

控制无人机完成建图后，用以下指令保存.ot（或者.bt，相较于.ot体积更小）格式的地图文件

```bash
rosrun octomap_server octomap_saver -f ~/EasonDrone/Reconstruction/EasonDrone_Mapping/map.ot
```

![image](doc/log/2024-03-11/%E6%97%A0%E6%A0%87%E9%A2%98.png)

参考：
- https://octomap.github.io/octomap/doc
- ⭐️ https://wiki.ros.org/octomap
- ⭐️ https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp
- ⭐️ https://groups.google.com/g/octomap/c/ZyfNzcuGlY0?pli=1

## 查看地图

### 方法1：octovis

也可以借助`octovis`工具查看

```bash
octovis ~/EasonDrone/Reconstruction/EasonDrone_Mapping/map.bt
```

### 方法2：rviz

一种方法是在`rviz`中查看

```bash
rosrun rviz rviz
rosrun octomap_server octomap_server_node ~/EasonDrone/Reconstruction/EasonDrone_Mapping/map.bt
```

添加`OccupancyGrid`，话题选择`/octomap_binary`，

![image](doc/log/2024-03-15/Snipaste_2024-03-15_14-38-27.png)

参考：
- [在ROS中将点云（PointCloud2）生成Octomap，rviz可视化显示](https://blog.csdn.net/qq_41816368/article/details/133929136)
- ⭐️ [octomap in rviz and occupancy grids in 3D maps](https://robotics.stackexchange.com/questions/41362/octomap-in-rviz-and-occupancy-grids-in-3d-maps)

## 加载地图

参考：
- ⭐ [how to use octomap_server?](https://answers.ros.org/question/361841/how-to-use-octomap_server/)
- ⭐ [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/launch/octomap_tracking_server.launch)

## Acknowledgement

Thanks for following packages:

- [global_planning](https://github.com/amov-lab/Prometheus/Modules/planning/global_planning)
