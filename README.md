# octomapping

A ROS package targeted for building 3D octomap on Premetheus P450 (Nano), suit for other kinds of UAV.

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Foctomapping.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/C%2B%2B-11-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)


> RealSense T265 is a tracking camera that is designed to be more optimal for Visual Odometry and SLAM (wider field of view and not using infrared light). It can do SLAM onboard as well as loop closure. However, this camera is not able to return RGB images (since it does not have a RGB camera onboard) and the depth returned is not as good as the D400 series (and can be a little trickier to get).

> Using both a RealSense D435i sensor and a RealSense T265 sensor can provide both the maps and the better quality visual odometry for developing a full SLAM system. The D435i used for the mapping, and the T265 for the tracking.


## OctoMap

![image](https://github.com/HuaYuXiao/octomapping/assets/117464811/b7b1213b-6ece-4e81-9ae2-f4e863a87571)

```bash
rosrun octomap_server octomap_saver -f map.bt
```


参考：
- https://octomap.github.io/octomap/doc
- https://wiki.ros.org/octomap

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

![Screenshot from 2024-03-04 06-39-57](https://github.com/HuaYuXiao/UAV-Dynamic-Obstacle-Avoidance/assets/117464811/fb10c834-d753-452b-b0a5-3b5b0b7bae20)

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


