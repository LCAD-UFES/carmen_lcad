#!/usr/bin/env python3
"""Publish TwistStamped records received from the ROS2 bridge over stdin."""
import json
import sys

import rospy
from geometry_msgs.msg import TwistStamped


def main():
    rospy.init_node('odometry_ros2_to_ros1', disable_signals=False)
    publisher = rospy.Publisher('/ackermann/odom_raw', TwistStamped, queue_size=200)
    print('READY', flush=True)
    for line in sys.stdin:
        if rospy.is_shutdown():
            break
        sec, nsec, frame, values = json.loads(line)
        msg = TwistStamped()
        msg.header.stamp = rospy.Time(sec, nsec)
        msg.header.frame_id = frame
        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
         msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = values
        publisher.publish(msg)
    rospy.signal_shutdown('ROS2 bridge disconnected')


if __name__ == '__main__':
    main()
