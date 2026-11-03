#!/usr/bin/env python3
"""Bidirectional bridge for the remaining CARMEN/SLAM topics."""
from collections import deque
import json
from pathlib import Path
import signal
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from remaining_protocol import OUTBOUND, INBOUND, message_type, encode_ros2


def main():
    rclpy.init()
    node = rclpy.create_node('remaining_ros2_bridge')
    endpoint = None
    reader = None
    stopping = threading.Event()
    try:
        endpoint = subprocess.Popen(
            ['/bin/bash', str(Path(__file__).resolve().with_name('run_remaining_ros1.sh')),
             str(node.declare_parameter('ros1_use_sim_time', False).value).lower()],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
        if endpoint.stdout.readline().strip() != 'READY':
            raise RuntimeError('ROS1 endpoint did not start')
        publishers = {topic: node.create_publisher(message_type(spec), topic, spec[2])
                      for topic, spec in INBOUND.items()}
        pending = {topic: deque(maxlen=spec[2]) for topic, spec in INBOUND.items()}
        lock = threading.Lock()
        failures = []

        def receive():
            try:
                for line in endpoint.stdout:
                    topic, values = json.loads(line)
                    with lock:
                        pending[topic].append(values)
            except Exception as error:
                failures.append(error)
            finally:
                if not stopping.is_set():
                    failures.append(RuntimeError('ROS1 endpoint disconnected'))

        reader = threading.Thread(target=receive, daemon=True)
        reader.start()

        def dispatch():
            if failures:
                raise RuntimeError(str(failures[0]))
            if endpoint.poll() is not None:
                raise RuntimeError('ROS1 endpoint exited')
            with lock:
                batches = {topic: list(queue) for topic, queue in pending.items()}
                for queue in pending.values():
                    queue.clear()
            for topic, records in batches.items():
                for values in records:
                    msg = message_type(INBOUND[topic])()
                    set_message_fields(msg, values)
                    publishers[topic].publish(msg)

        def forward(msg, info, topic):
            if topic in INBOUND:
                for peer in node.get_publishers_info_by_topic(topic):
                    if (peer.node_name == node.get_name() and peer.node_namespace == node.get_namespace()
                            and bytes(peer.endpoint_gid) == bytes(info['publisher_gid']['data'])):
                        return
            endpoint.stdin.write(json.dumps([topic, encode_ros2(msg)], separators=(',', ':'))+'\n')
            endpoint.stdin.flush()

        subscriptions = []
        for topic, spec in OUTBOUND.items():
            qos = QoSProfile(depth=spec[2], reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if spec[3]
                             else DurabilityPolicy.VOLATILE)
            def make_callback(topic):
                def callback(msg, info):
                    forward(msg, info, topic)
                return callback
            subscriptions.append(node.create_subscription(message_type(spec), topic, make_callback(topic), qos))
        node.create_timer(.005, dispatch)
        node.get_logger().info('ROS2 <-> ROS1 LiDAR, GPS, clock and SLAM bridge ready')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stopping.set()
        if endpoint is not None:
            if endpoint.stdin:
                try: endpoint.stdin.close()
                except BrokenPipeError: pass
            try: endpoint.wait(timeout=3)
            except subprocess.TimeoutExpired:
                endpoint.terminate()
                try: endpoint.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    endpoint.kill(); endpoint.wait()
        if reader:
            reader.join(timeout=2)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
