#!/usr/bin/env python3
"""Bridge IMU, dynamic GPS receivers and retained static TF to ROS1."""
import json
from pathlib import Path
import subprocess
import signal
import re

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    rclpy.init()
    node = rclpy.create_node('sensors_ros2_to_ros1')
    sink = None
    try:
        sink = subprocess.Popen(
            ['/bin/bash', str(Path(__file__).resolve().with_name('run_sensors_ros1_sink.sh'))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
        if sink.stdout.readline().strip() != 'READY':
            raise RuntimeError('ROS1 publisher failed to initialize')
        sink.stdout.close()

        def send(topic, data):
            if sink.poll() is not None:
                raise RuntimeError('ROS1 publisher exited')
            sink.stdin.write(json.dumps([topic, data], separators=(',', ':')) + '\n')
            sink.stdin.flush()

        subscriptions = {}

        def subscribe(topic, message_type, depth, retained=False):
            if topic in subscriptions:
                return
            send(topic, None)
            qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if retained
                             else DurabilityPolicy.VOLATILE)
            subscriptions[topic] = node.create_subscription(
                message_type, topic,
                lambda msg: send(topic, message_to_ordereddict(msg)), qos)

        subscribe('/imu_raw', Imu, 2000)
        subscribe('/gps/heading_raw', QuaternionStamped, 50)
        subscribe('/tf_static', TFMessage, 100, retained=True)
        for nr in node.declare_parameter('gps_nrs', [1, 2]).value:
            subscribe('/gps/xyz_raw/gps_'+str(nr), Odometry, 50)

        def discover_gps():
            for topic, types in node.get_topic_names_and_types():
                if (re.fullmatch(r'/gps/xyz_raw/gps_-?\d+', topic) and
                        'nav_msgs/msg/Odometry' in types):
                    subscribe(topic, Odometry, 50)

        node.create_timer(0.1, discover_gps)

        def check_sink():
            if sink.poll() is not None:
                raise RuntimeError('ROS1 publisher exited')

        node.create_timer(1.0, check_sink)
        node.get_logger().info('ROS2 -> ROS1: IMU, GPS and static TF ready')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward a second shutdown signal.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if sink is not None:
            if sink.stdin:
                try:
                    sink.stdin.close()
                except BrokenPipeError:
                    pass
            try:
                sink.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sink.terminate()
                try:
                    sink.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    sink.kill()
                    sink.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
