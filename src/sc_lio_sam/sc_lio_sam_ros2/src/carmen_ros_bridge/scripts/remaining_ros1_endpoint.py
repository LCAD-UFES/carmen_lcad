#!/usr/bin/env python3
"""ROS1 endpoint; caller identity prevents initialpose feedback loops."""
import json
import sys
import threading
import rospy
from remaining_protocol import OUTBOUND, INBOUND, message_type, encode_ros1, fill_ros1


def main():
    rospy.set_param('/use_sim_time', sys.argv[1] == 'true')
    rospy.init_node('remaining_ros2_bridge')
    lock = threading.Lock()
    publishers = {topic: rospy.Publisher(topic, message_type(spec), queue_size=spec[2], latch=spec[3])
                  for topic, spec in OUTBOUND.items()}

    def forward(msg, topic):
        if msg._connection_header.get('callerid') == rospy.get_name():
            return
        with lock:
            sys.stdout.write(json.dumps([topic, encode_ros1(msg)], separators=(',', ':'))+'\n')
            sys.stdout.flush()

    with lock:
        print('READY', flush=True)
    subscribers = [rospy.Subscriber(topic, message_type(spec), forward, callback_args=topic,
                                   queue_size=spec[2]) for topic, spec in INBOUND.items()]
    for line in sys.stdin:
        if rospy.is_shutdown():
            break
        topic, values = json.loads(line)
        spec = OUTBOUND[topic]
        publishers[topic].publish(fill_ros1(message_type(spec)(), values))
    rospy.signal_shutdown('ROS2 bridge disconnected')


if __name__ == '__main__':
    main()
