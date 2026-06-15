#!/usr/bin/env python3
"""
odom_to_tf.py
odometry 메시지(nav_msgs/Odometry)를 구독해서
odom → base_footprint 동적 TF를 publish.

타임스탬프 단조 증가 필터: 이전보다 오래된 메시지는 무시 → TF_OLD_DATA 방지
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.last_stamp_ns = -1
        self.sub = self.create_subscription(
            Odometry, 'odometry', self.odom_cb, 10)

    def odom_cb(self, msg: Odometry):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            return  # 오래된 메시지 무시 → TF_OLD_DATA 방지
        self.last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id       # odom
        t.child_frame_id = msg.child_frame_id          # base_footprint
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
