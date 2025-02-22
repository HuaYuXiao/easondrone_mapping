# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## TODO
- real multi-thread
- const cast
- timeout
- ICP multi-thread

## v2.1.8 - 2025-01-04
- remove `PointCloud` data around center within radius 0.4m

## v2.1.7 - 2024-12-27
- [new feature] optional ICP algorithm to concatinate `PointCloudT`s
- improve safety and efficiency for initialization
- replace `ros::Duration.sleep` with `std::this_thread::sleep_for`

## v2.1.6 - 2024-12-25
- [new feature] support for random num of pointcloud input sources
- [bug fix] a single `tf::TransformListener` is sufficient
- improve safety and efficiency for buffer process
- replace `boost::thread` with `std::thread`

## v2.1.5 - 2024-12-24
- [upgrade] apply multi-thread to process several pointclouds parallely
- [upgrade] store pointclouds with `std::deque`

## v2.1.4 - 2024-12-24
- [upgrade] set `queue_size` of `Subscriber` to `boost::thread::hardware_concurrency()`
- [upgrade] check if both `frame_id`s are the same
- [upgrade] configurable `tf_duration`

## v2.1.3 - 2024-08-23
- [new feature] 转发lidar_frame坐标系下的点云到world坐标系下

## v2.1.2
- import `message_filters::sync_policies::ApproximateTime` for merge

## v2.1.1
- publish pose

## v2.1.0
- merge 2 `PointCloud2`s, with fixed `frame_id = "base_link"`
