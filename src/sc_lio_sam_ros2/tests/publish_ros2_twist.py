#!/usr/bin/env python3
"""Exercise every TwistStamped field across the compatibility bridge."""
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node('odometry_bridge_fixture')
pub = node.create_publisher(TwistStamped, '/ackermann/odom_raw',
                            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
try:
    deadline = time.monotonic() + 10
    while pub.get_subscription_count() == 0:
        assert time.monotonic() < deadline, 'No ROS2 bridge subscriber'
        rclpy.spin_once(node, timeout_sec=.1)
    for i in range(5):
        msg = TwistStamped()
        msg.header.stamp.sec = 123+i
        msg.header.stamp.nanosec = 987654321
        msg.header.frame_id = 'test_frame_á'
        msg.twist.linear.x = 1.25+i
        msg.twist.linear.y = -2.5
        msg.twist.linear.z = 3.125
        msg.twist.angular.x = -4.75
        msg.twist.angular.y = 5.0625
        msg.twist.angular.z = -6.5
        pub.publish(msg)
        time.sleep(.1)
    time.sleep(.5)
finally:
    node.destroy_node()
    rclpy.shutdown()
