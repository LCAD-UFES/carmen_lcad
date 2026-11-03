#!/usr/bin/env python3
"""ROS1 endpoint for IMU, raw GPS and static transforms."""
import json
import re
import sys
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from tf2_msgs.msg import TFMessage


def fill_message(message, values):
    for field, value in values.items():
        if field == 'stamp':
            message.stamp = rospy.Time(value['sec'], value['nanosec'])
        elif isinstance(value, dict):
            fill_message(getattr(message, field), value)
        else:
            setattr(message, field, value)
    return message


def main():
    rospy.init_node('sensors_ros2_to_ros1')
    publishers = {
        '/imu_raw': rospy.Publisher('/imu_raw', Imu, queue_size=2000),
        '/gps/heading_raw': rospy.Publisher('/gps/heading_raw', QuaternionStamped, queue_size=50),
        '/tf_static': rospy.Publisher('/tf_static', TFMessage, queue_size=100, latch=True)}
    types = {'/imu_raw': Imu, '/gps/heading_raw': QuaternionStamped}
    static_transforms = {}
    print('READY', flush=True)
    for line in sys.stdin:
        if rospy.is_shutdown():
            break
        topic, data = json.loads(line)
        if topic not in publishers:
            if re.fullmatch(r'/gps/xyz_raw/gps_-?\d+', topic) is None:
                raise ValueError('Unexpected sensor topic: '+topic)
            types[topic] = Odometry
            publishers[topic] = rospy.Publisher(topic, Odometry, queue_size=50)
        if data is None:
            continue
        if topic == '/tf_static':
            for transform in data['transforms']:
                msg = fill_message(TransformStamped(), transform)
                static_transforms[msg.child_frame_id] = msg
            # Retain every child frame for late ROS1 subscribers.
            publishers[topic].publish(TFMessage(transforms=list(static_transforms.values())))
        else:
            publishers[topic].publish(fill_message(types[topic](), data))
    rospy.signal_shutdown('ROS2 bridge disconnected')


if __name__ == '__main__':
    main()
