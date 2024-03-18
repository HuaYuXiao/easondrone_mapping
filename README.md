# ![image](Img/logo_rt_200.png) uav_octomapping

A ROS package to build 3D octomap with 2D lidar on Premetheus P450 (Nano), also suit for other kinds of UAV. 分为实物和仿真两部分。

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FUAV_octomapping.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/C%2B%2B-11-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)


> RealSense T265 is a tracking camera that is designed to be more optimal for Visual Odometry and SLAM (wider field of view and not using infrared light). It can do SLAM onboard as well as loop closure. However, this camera is not able to return RGB images (since it does not have a RGB camera onboard) and the depth returned is not as good as the D400 series (and can be a little trickier to get).

> Using both a RealSense D435i sensor and a RealSense T265 sensor can provide both the maps and the better quality visual odometry for developing a full SLAM system. The D435i used for the mapping, and the T265 for the tracking.


## 实物实验

### 1. 下载源码

```bash
cd ~/catkin_ws
```

```bash
git clone https://github.com/HuaYuXiao/uav_octomapping.git
```


### 2. 编译安装

```bash
catkin_make install --pkg=uav_octomapping
```

```bash
gedit ~/.bashrc
```

在文件末尾加上：`source ~/catkin_ws/devel/setup.bash`（已经添加过的跳过）。


### 3. 建立地图

```bash
roslaunch uav_octomapping octomapping.launch
```


### 4. 保存地图

> You are trying to invoke `octomap_saver` as an argument to the `octomap_server` node. However, `octomap_saver` is a node of its own, so you only have to start it from a separate terminal while `octomap_server` is running. Check the documentation at http://wiki.ros.org/octomap_server#octomap_saver


控制无人机完成建图后，用以下指令保存.bt（相较于.ot体积更小）格式的地图文件，默认保存到~/下。

```bash
rosrun octomap_server octomap_saver -f map.bt
```

![image](Log/2024-03-11/%E6%97%A0%E6%A0%87%E9%A2%98.png)


参考：
- https://octomap.github.io/octomap/doc
- ⭐️ https://wiki.ros.org/octomap
- ⭐️ https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp
- ⭐️ https://groups.google.com/g/octomap/c/ZyfNzcuGlY0?pli=1


### 5. 查看地图

#### 方法1：rviz

一种方法是在`rviz`中查看，

```bash
roscore
```

新开一个标签，

```bash
rosrun rviz rviz
```

新开一个标签，

```bash
rosrun octomap_server octomap_server_node map.bt
```

添加`OccupancyGrid`，话题选择`/octomap_binary`。

![image](Log/2024-03-15/Snipaste_2024-03-15_14-38-27.png)

参考：
- [在ROS中将点云（PointCloud2）生成Octomap，rviz可视化显示](https://blog.csdn.net/qq_41816368/article/details/133929136)
- ⭐️ [octomap in rviz and occupancy grids in 3D maps](https://robotics.stackexchange.com/questions/41362/octomap-in-rviz-and-occupancy-grids-in-3d-maps)

#### 方法2：octovis

也可以借助`octovis`工具查看。

```bash
octovis map.bt
```

**NOTICE**：
- 注意要将`map.bt`放到指定文件夹下，否则无法读取。
- 地图文件较大，加载需要一些时间，请耐心等待。


### 6. 加载地图

运用细节可以参考另一个仓库：https://github.com/HuaYuXiao/uav_navigation/launch/navigation.launch

参考：
- ⭐ [how to use octomap_server?](https://answers.ros.org/question/361841/how-to-use-octomap_server/)
- ⭐ [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/launch/octomap_tracking_server.launch)



## 仿真实验

仿真实验也采用Prometheus 450，环境安装配置请参考https://www.bilibili.com/video/BV16S4y1E7br

```bash
'/home/hyx020222/Prometheus/Scripts/simulation/px4_gazebo_sitl_test/px4_sitl_indoor.sh'
```







## Cartographer

> Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

开始建图

```bash
roslaunch cartographer_ros ~/cartographer_ws/src/cartographer_ros/cartographer_ros/launch/demo_backpack_3d.launch
```

结束建图

```bash
rosservice call /finish_trajectory 0
```

```bash
rosservice call /write_state "{filename: '~/map.pbstream'}"
```

由于板载计算机性能较差，因此只开展了仿真实验。

```bash
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
```

![image](https://github.com/HuaYuXiao/UAV-Dynamic-Obstacle-Avoidance/assets/117464811/fb10c834-d753-452b-b0a5-3b5b0b7bae20)

参考：
- ⭐[手把手教你编译cartographer](https://www.bilibili.com/video/BV19P4y1X7Hj)
- [Cartographer ROS Integration](https://google-cartographer-ros.readthedocs.io/en/latest/)
- [cartographer-project/cartographer_ros](https://github.com/cartographer-project/cartographer_ros)



## ORB_SLAM3


参考：
- ⭐[【无人机自主导航5 SLAM】Intel Realsense T265C双目相机实现ORB-SLAM3](https://dgzc.ganahe.top/ganahe/2021/wrjzzdhsjirtsmxj.html)
- ⭐[ubuntu18.04 从0开始运行ORB_SLAM2](https://www.bilibili.com/video/BV1hQ4y127xJ)
- Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM, IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021.



## rtabmap

**NOTICE**: Installation of `rtabmap` is required for using this launch file

<!--
```bash
sudo apt-get install ros-melodic-rtabmap-ros
```
-->

```bash
roslaunch realsense2_camera rs_rtabmap.launch
```

**2024年2月28日更新**：`rtabmap`涉及到RGB-D，该机器不具备直接获取深度数据的能力，因此该方案废弃。

参考：
- [Introduction to Intel® RealSense™ Visual SLAM and the T265 Tracking Camera](https://dev.intelrealsense.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera)
- [Intel RealSense 3D Camera for Robotics & SLAM (with code)](https://www.robotsforroboticists.com/realsense-usage-robotics-slam/)
- [SLAM模块(Prometheus/Modules/slam)](https://docs.amovlab.com/prometheuswiki/#/src/P450%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C/%E8%BD%AF%E4%BB%B6%E4%BB%8B%E7%BB%8D?id=slam%e6%a8%a1%e5%9d%97prometheusmodulesslam-)



## 谢辞
- 感谢陈亮名副教授提供的技术指导😊！
- 感谢哈工深MASLAB提供的场地支持😊！
- 感谢刘嘉雯、崔宝艺、李奥淇、方尧等师兄师姐的支持😊！
