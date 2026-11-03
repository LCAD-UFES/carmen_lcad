#!/usr/bin/env python3
"""Bridge only ROS2 /ackermann/odom_raw to ROS1 using an anonymous pipe."""
import json
from pathlib import Path
import subprocess
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('odometry_ros2_to_ros1')
    sink = None
    try:
        sink = subprocess.Popen(
            ['/bin/bash', str(Path(__file__).resolve().with_name('run_ros1_sink.sh'))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
        if sink.stdout.readline().strip() != 'READY':
            raise RuntimeError('ROS1 publisher failed to initialize')
        sink.stdout.close()

        def forward(msg):
            if sink.poll() is not None:
                raise RuntimeError('ROS1 publisher exited')
            t = msg.twist
            record = [msg.header.stamp.sec, msg.header.stamp.nanosec,
                      msg.header.frame_id,
                      [t.linear.x, t.linear.y, t.linear.z,
                       t.angular.x, t.angular.y, t.angular.z]]
            sink.stdin.write(json.dumps(record, separators=(',', ':')) + '\n')
            sink.stdin.flush()

        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(TwistStamped, '/ackermann/odom_raw', forward, qos)

        def check_sink():
            if sink.poll() is not None:
                raise RuntimeError('ROS1 publisher exited')

        node.create_timer(1.0, check_sink)
        node.get_logger().info('ROS2 -> ROS1: /ackermann/odom_raw ready')
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
