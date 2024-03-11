#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform

class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos_profile = QoSProfile(depth=10)
        self.real_odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile)

    def odom_callback(self, msg):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'odom'

        transform_stamped.transform = self.get_odom_to_map_transform(msg)

        self.tf_broadcaster.sendTransform(transform_stamped)

    def get_odom_to_map_transform(self, odom_msg):
        transform = Transform()

        # Extracting position and orientation from Odometry message
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation

        transform.translation.x = p.x
        transform.translation.y = p.y
        transform.translation.z = p.z
        transform.rotation.x = q.x
        transform.rotation.y = q.y
        transform.rotation.z = q.z
        transform.rotation.w = q.w

        return transform

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
