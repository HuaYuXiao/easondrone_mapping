#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2

class LidarDataTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pub_pcl_world', anonymous=True)

        # Set up the transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the PointCloud2 topic
        self.pc_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)

        # Publisher for transformed point cloud data
        self.pc_publisher = rospy.Publisher('/velodyne_points_world', PointCloud2, queue_size=10)

    def point_cloud_callback(self, msg):
        try:
            # Look up the transform from 'world' to 'lidar_link'
            transform = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Transform the point cloud to the 'world' frame
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform)

            # Publish the transformed point cloud
            self.pc_publisher.publish(transformed_cloud)

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")
        except Exception as e:
            rospy.logerr(f"Error transforming point cloud: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = LidarDataTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass
