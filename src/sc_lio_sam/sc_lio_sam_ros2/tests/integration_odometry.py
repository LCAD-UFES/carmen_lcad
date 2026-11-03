#!/usr/bin/env python3
"""Run with Noetic sourced; starts isolated ROS1, ROS2 and IPC services."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client

workspace = Path(__file__).resolve().parents[1]
carmen = Path(os.environ.get('CARMEN_HOME', '/home/eduardopia/carmen_lcad'))
logs = Path(tempfile.mkdtemp(prefix='ros2-odometry-test-'))
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11441', ROS_IP='127.0.0.1',
                  ROS_DOMAIN_ID='141', CENTRALHOST='localhost:14841',
                  ROS_HOME=str(logs/'ros'), ROS_LOG_DIR=str(logs/'logs'))
processes = []

def start(name, args):
    with (logs/(name+'.log')).open('w') as output:
        process = subprocess.Popen(args, cwd=carmen/'bin', stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process

try:
    start('master', ['roscore', '-p', '11441'])
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    for _ in range(100):
        try:
            master.getPid('/test')
            break
        except OSError:
            time.sleep(0.1)
    else:
        raise RuntimeError('ROS master startup failed')
    start('central', ['./central', '-p14841'])
    time.sleep(1)
    start('params', ['./param_daemon', '../data/iara/parameters/carmen.ini'])
    time.sleep(2)
    import rospy
    from geometry_msgs.msg import TwistStamped
    rospy.init_node('odometry_integration', disable_signals=True)
    received = []
    sub = rospy.Subscriber('/ackermann/odom_raw', TwistStamped, received.append, queue_size=200)
    bridge = start('odometry', [str(workspace/'run_odometry_ros1.sh'), 'base_frame_id:=test_base'])
    for _ in range(150):
        if bridge.poll() is not None:
            raise RuntimeError('Launch exited; see '+str(logs))
        if sub.get_num_connections() == 1:
            time.sleep(2)
            break
        time.sleep(.1)
    else:
        raise RuntimeError('No ROS1 publisher; see '+str(logs))
    publisher = start('source', ['/tmp/publish_ipc_odometry'])
    assert publisher.wait(timeout=10) == 0
    for _ in range(50):
        if len(received) == 20:
            break
        time.sleep(.1)
    assert len(received) == 20, (len(received), str(logs))
    for i, msg in enumerate(received):
        assert msg.header.frame_id == 'test_base'
        assert msg.header.stamp.to_nsec() == int((1793710000.125 + i*.125)*1e9)
        assert msg.twist.linear.x == -2.5+i*.25
        assert msg.twist.angular.z == -.3+i*.01
        assert msg.twist.linear.y == msg.twist.linear.z == 0
        assert msg.twist.angular.x == msg.twist.angular.y == 0
    assert bridge.poll() is None
    print('PASS: 20 IPC -> ROS2 -> ROS1 messages; exact stamps, frame, speed, steering and zeros.')
    received.clear()
    fixture = start('ros2_fields', ['env', '-u', 'PYTHONPATH', '-u', 'LD_LIBRARY_PATH',
        '-u', 'ROS_DISTRO', '-u', 'ROS_VERSION', '-u', 'ROS_PACKAGE_PATH',
        '-u', 'CMAKE_PREFIX_PATH', '/bin/bash', '--noprofile', '--norc', '-c',
        'source /opt/ros/lyrical/setup.bash; exec /usr/bin/python3 "$1"',
        'fixture', str(workspace/'tests/publish_ros2_twist.py')])
    assert fixture.wait(timeout=15) == 0
    time.sleep(.5)
    assert len(received) == 5, len(received)
    for i, msg in enumerate(received):
        assert msg.header.stamp.secs == 123+i
        assert msg.header.stamp.nsecs == 987654321
        assert msg.header.frame_id == 'test_frame_á'
        t = msg.twist
        assert [t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z] == [1.25+i,-2.5,3.125,-4.75,5.0625,-6.5]
    print('PASS: all six components, nanoseconds and UTF-8 frame preserved.')
    os.killpg(bridge.pid, signal.SIGINT)
    bridge.wait(timeout=15)
    received.clear()
    bridge = start('disabled', [str(workspace/'run_odometry_ros1.sh'), 'publish_ackermann:=false'])
    time.sleep(4)
    publisher = start('disabled_source', ['/tmp/publish_ipc_odometry'])
    assert publisher.wait(timeout=10) == 0
    time.sleep(1)
    assert bridge.poll() is None
    assert not received, 'publish_ackermann=false still publishes'
    print('PASS: publish_ackermann=false suppresses odometry.')
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    print('Logs:', logs)

for name in ('odometry', 'disabled'):
    log = (logs/(name+'.log')).read_text()
    assert 'Traceback' not in log and '[ERROR]' not in log, str(logs)
print('PASS: both launch runs shut down cleanly.')
