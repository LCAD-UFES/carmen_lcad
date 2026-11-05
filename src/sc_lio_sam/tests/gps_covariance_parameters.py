#!/usr/bin/env python3
"""Check GPS-quality parameters against real IPC-to-ROS2 Odometry messages.

Source setup.bash first. Uses isolated IPC port 14848 and ROS domain 148,
the IARA param_daemon configuration, and temporary logs/build products.
"""
import argparse
import json
import math
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import tempfile
import time

import yaml


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--executable', type=Path,
                    help='Override the installed carmen_to_ros_gps_xyz executable.')
args = parser.parse_args()
carmen = Path(os.environ.get('CARMEN_HOME', str(Path.home() / 'carmen_lcad'))).resolve()
workspace = Path(__file__).resolve().parents[1]
executable = args.executable or workspace / 'install/carmen_ros_bridge/lib/carmen_ros_bridge/carmen_to_ros_gps_xyz'
assert executable.is_file(), 'Missing GPS executable: ' + str(executable)
logs = Path(tempfile.mkdtemp(prefix='gps-covariance-parameters-'))
print('Results:', logs, flush=True)
os.environ.update(CENTRALHOST='localhost:14848', ROS_DOMAIN_ID='148',
                  ROS_LOG_DIR=str(logs / 'ros'))
processes = []


def start(name, command, **kwargs):
    output = (logs / (name + '.log')).open('w')
    process = subprocess.Popen(command, cwd=carmen / 'bin', stdout=output,
                               stderr=subprocess.STDOUT, start_new_session=True, **kwargs)
    output.close()
    processes.append(process)
    return process


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def check_invalid_parameters():
    invalid = [
        ('zero', {'gps_sigma_xy_quality_1': 0.0}, 'gps_sigma_xy_quality_1'),
        ('negative', {'gps_sigma_xy_quality_2': -1.0}, 'gps_sigma_xy_quality_2'),
        ('nan', {'gps_sigma_xy_quality_4': math.nan}, 'gps_sigma_xy_quality_4'),
        ('infinity', {'gps_sigma_xy_quality_5': math.inf}, 'gps_sigma_xy_quality_5'),
        ('zero_multiplier', {'gps_sigma_z_multiplier': 0.0}, 'gps_sigma_z_multiplier'),
        ('overflow', {'gps_sigma_xy_default': 1.0e200}, 'gps_sigma_xy_default'),
        ('vertical_overflow', {'gps_sigma_z_multiplier': 1.0e200}, 'gps_sigma_z_multiplier'),
    ]
    # No central runs yet: invalid settings must be rejected before IPC startup.
    for label, parameters, expected_name in invalid:
        filename = logs / ('invalid-' + label + '.yaml')
        filename.write_text(yaml.safe_dump({'carmen_to_ros_gps_xyz': {'ros__parameters': parameters}}))
        result = subprocess.run([str(executable), '--ros-args', '--params-file', str(filename)],
                                capture_output=True, text=True, timeout=5)
        text = result.stdout + result.stderr
        (logs / ('invalid-' + label + '.log')).write_text(text)
        assert result.returncode != 0, label
        assert expected_name in text, (label, text)
        assert 'Invalid GPS parameters' in text, (label, text)
    print('PASS invalid/nonfinite/overflow parameter rejection before IPC', flush=True)


def wait_for_central(process):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Central exited'
        try:
            with socket.create_connection(('localhost', 14848), timeout=.1):
                return
        except OSError:
            time.sleep(.05)
    raise AssertionError('Central did not open IPC port 14848')


subprocess.run([
    'g++', '-std=c++17', str(Path(__file__).with_name('publish_ipc_gps_quality.cpp')),
    '-I' + str(carmen / 'include'), '-L' + str(carmen / 'lib'),
    '-Wl,--start-group', '-lglobal', '-lipc', '-Wl,--end-group', '-lz', '-lm',
    '-o', str(logs / 'publish_ipc_gps_quality')], check=True)

import rclpy
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('gps_covariance_parameter_test')
received = {}
subscriptions = []
for receiver in (1, 2, 7):
    subscriptions.append(node.create_subscription(
        Odometry, '/gps/xyz_raw/gps_' + str(receiver),
        lambda message, receiver=receiver: received.__setitem__(
            (receiver, round(message.pose.pose.position.x)), message), 100))


def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline - time.monotonic())))


def send_and_wait(publisher, bridge, receiver, sequence, quality):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        assert bridge.poll() is None, 'GPS bridge exited'
        assert publisher.poll() is None, 'IPC fixture exited'
        publisher.stdin.write(f'{sequence} {receiver} {quality}\n')
        publisher.stdin.flush()
        spin(.1)
        if (receiver, sequence) in received:
            return received[(receiver, sequence)]
    raise AssertionError(f'No GPS Odometry for receiver={receiver}, quality={quality}')


def check_profile(label, overrides, sigmas, z_multiplier, sequence_base):
    command = [str(executable), '--ros-args']
    for name, value in overrides.items():
        command.extend(['-p', name + ':=' + str(value)])
    bridge = start('bridge-' + label, command)
    # Repeated labelled samples cover discovery and the node's lazy publishers.
    for receiver in (1, 2, 7):
        send_and_wait(publisher, bridge, receiver, sequence_base + receiver, 4)

    for index, quality in enumerate((1, 2, 4, 5, 9)):
        sigma = sigmas[quality]
        for receiver in (1, 2, 7):
            sequence = sequence_base + 100 + index * 10 + receiver
            message = send_and_wait(publisher, bridge, receiver, sequence, quality)
            assert message.header.frame_id == 'utm'
            assert message.child_frame_id == 'gps_' + str(receiver)
            assert message.pose.pose.position.y == -sequence
            expected = {0: sigma ** 2, 7: sigma ** 2, 14: (sigma * z_multiplier) ** 2}
            for diagonal, variance in expected.items():
                actual = message.pose.covariance[diagonal]
                assert math.isclose(actual, variance, rel_tol=1e-12, abs_tol=1e-15), (
                    label, receiver, quality, diagonal, actual, variance)

    for receiver in (1, 2, 7):
        sequence = sequence_base + 200 + receiver
        for _ in range(5):
            publisher.stdin.write(f'{sequence} {receiver} 0\n')
            publisher.stdin.flush()
            spin(.05)
        assert (receiver, sequence) not in received, 'Invalid fix was published'
    stop(bridge)
    print('PASS', label, 'all quality branches, XYZ variances and invalid-fix filtering', flush=True)


publisher = None
try:
    with socket.socket() as probe:
        probe.bind(('localhost', 14848))
    check_invalid_parameters()
    central = start('central', ['./central', '-p14848'])
    wait_for_central(central)
    params = start('param_daemon', ['./param_daemon', str(carmen / 'data/iara/parameters/carmen.ini')])
    time.sleep(1)
    assert params.poll() is None, 'IARA param_daemon exited'
    fixture_log = (logs / 'fixture-stderr.log').open('w')
    publisher = subprocess.Popen([str(logs / 'publish_ipc_gps_quality')],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=fixture_log,
                                 text=True, bufsize=1, start_new_session=True)
    fixture_log.close()
    processes.append(publisher)
    selector = selectors.DefaultSelector()
    selector.register(publisher.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + 5
    ready = False
    fixture_output = b''
    while time.monotonic() < deadline:
        assert publisher.poll() is None, 'IPC fixture exited during initialization'
        if selector.select(timeout=.1):
            fixture_output += os.read(publisher.stdout.fileno(), 4096)
            if b'READY' in fixture_output.splitlines():
                ready = True
                break
    selector.close()
    assert ready, 'IPC fixture did not become ready'
    check_profile('defaults', {}, {1: 5.0, 2: 1.0, 4: .02, 5: .20, 9: 50.0}, 3.0, 1000)
    overrides = dict(gps_sigma_xy_quality_1=2.0, gps_sigma_xy_quality_2=.5,
                     gps_sigma_xy_quality_4=.03, gps_sigma_xy_quality_5=.4,
                     gps_sigma_xy_default=8.0, gps_sigma_z_multiplier=2.0)
    check_profile('overrides', overrides, {1: 2.0, 2: .5, 4: .03, 5: .4, 9: 8.0}, 2.0, 2000)
    (logs / 'result.json').write_text(json.dumps(dict(
        passed=True, profiles=['defaults', 'overrides'], receivers=[1, 2, 7],
        qualities=[0, 1, 2, 4, 5, 9], invalid_parameter_cases=7), indent=2) + '\n')
finally:
    for process in reversed(processes):
        stop(process)
    if publisher is not None:
        publisher.stdin.close()
        publisher.stdout.close()
    node.destroy_node()
    rclpy.shutdown()
