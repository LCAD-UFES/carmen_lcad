#!/usr/bin/env python3
"""Run a vehicle process file with isolated IPC, ROS and disposable map output."""
import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import psutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('vehicle', choices=('iara', 'argos'))
parser.add_argument('--allow-rviz-shutdown-crash', action='store_true')
parser.add_argument('--require-nvidia-rendering', action='store_true')
args = parser.parse_args()
carmen = Path(os.environ.get('CARMEN_HOME', str(Path.home() / 'carmen_lcad')))
source = carmen / ('data/iara/process/process-playback_iara-sc.ini' if args.vehicle == 'iara'
                   else 'bin/argos/process-argos-playback-lio-sam.ini')
logs = Path(tempfile.mkdtemp(prefix='proccontrol-' + args.vehicle + '-'))
print('Results:', logs, flush=True)
# Preserve module commands and startup flags; redirect their log files only.
config = re.sub(r'(?<!>)>\s*([\w.-]+\.txt)',
                lambda match: '> ' + str(logs / match.group(1)), source.read_text())
process_file = logs / source.name
process_file.write_text(config)
env = dict(os.environ, CENTRALHOST='localhost:14849', ROS_DOMAIN_ID='149',
           ROS_LOG_DIR=str(logs / 'ros'))
processes = []

def start(name, command):
    with (logs / (name + '.log')).open('w') as output:
        process = subprocess.Popen(command, cwd=carmen / 'bin', env=env,
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process

controller = None
rviz_shutdown_crashes = []
try:
    start('central', ['./central', '-p14849'])
    time.sleep(1)
    controller = start('proccontrol', ['./proccontrol', '-SET', 'SC_LIO_SAM_MAP_PATH=' + str(logs / 'map'),
                                      str(process_file)])
    time.sleep(15)
    assert controller.poll() is None, 'proccontrol exited'
    children = psutil.Process(controller.pid).children(recursive=True)
    commands = [p.cmdline() for p in children]
    assert not any(any('roslaunch' in word or 'roscore' in word or 'ros1_bridge' in word
                       for word in command) for command in commands), commands
    for executable in ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
                       'lio_sam_featureExtraction', 'lio_sam_mapOptmization'):
        assert any(command and Path(command[0]).name == executable for command in commands), executable
    assert any(any('rviz2' in word for word in command) for command in commands), 'RViz2 missing'
    monitored = [p for p in children if p.name().startswith('lio_sam_') or p.name() == 'rviz2']
    rviz_process = next(p for p in monitored if p.name() == 'rviz2')
    rendering_maps = Path('/proc/' + str(rviz_process.pid) + '/maps').read_text()
    nvidia_loaded = 'libnvidia-glcore' in rendering_maps
    (logs / 'rendering.txt').write_text('NVIDIA OpenGL loaded: ' + str(nvidia_loaded) + '\n')
    if args.require_nvidia_rendering:
        assert nvidia_loaded, 'RViz2 did not load NVIDIA OpenGL'
        assert rviz_process.environ().get('LIBGL_ALWAYS_SOFTWARE', '0') not in ('1', 'true')
    (logs / 'processes.txt').write_text('\n'.join(' '.join(command) for command in commands))
    subprocess.run(['/tmp/playback_command', 't 30:90'], env=env, check=True, stdout=subprocess.DEVNULL)
    for _ in range(14):
        time.sleep(5)
        assert controller.poll() is None, 'proccontrol exited'
        assert all(p.is_running() and p.status() != psutil.STATUS_ZOMBIE for p in monitored), 'SLAM or RViz2 exited during playback'
    subprocess.run(['/tmp/playback_command', 'stop'], env=env, check=True, stdout=subprocess.DEVNULL)
    runtime_log = '\n'.join(p.read_text(errors='replace') for p in logs.glob('*.txt'))
    assert 'process has died' not in runtime_log and 'Segmentation fault' not in runtime_log
    (logs / 'before_shutdown.log').write_text(runtime_log)
finally:
    if controller is not None and controller.poll() is None:
        # Stop supervision before letting each ROS launch finish saving its map.
        controller.send_signal(signal.SIGSTOP)
        children = psutil.Process(controller.pid).children(recursive=True)
        launches = [p for p in children if 'ros2' in p.name() and 'launch' in p.cmdline()]
        for process in launches:
            process.send_signal(signal.SIGINT)
        _, alive = psutil.wait_procs(launches, timeout=35)
        for process in reversed(children):
            try:
                if process.is_running():
                    process.send_signal(signal.SIGINT)
            except psutil.NoSuchProcess:
                pass
        _, alive = psutil.wait_procs(children, timeout=5)
        for process in alive:
            try:
                process.kill()
            except psutil.NoSuchProcess:
                pass
        controller.kill()
        controller.wait()
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
text = '\n'.join(p.read_text(errors='replace') for p in logs.glob('*.txt'))
count = text.count('[PUBLISH_GLOBAL]')
assert count > 20, ('Too few poses', count, str(logs))
rviz_shutdown_started = False
for line in text.splitlines():
    if re.search(r"\[rviz2-\d+\]: sending signal 'SIGINT'", line):
        rviz_shutdown_started = True
    if 'process has died' in line or 'Segmentation fault' in line:
        if (args.allow_rviz_shutdown_crash and rviz_shutdown_started and '[rviz2-' in line
                and 'exit code -11' in line):
            rviz_shutdown_crashes.append(line)
        else:
            raise AssertionError(line)
assert 'Saving map to pcd files completed' in text, 'Map save did not complete'
assert any('lio_sam_mapOptmization-' in line and 'process has finished cleanly' in line
           for line in text.splitlines()), 'Mapper did not shut down cleanly'
for filename in ('trajectory.pcd', 'cloudCorner.pcd', 'cloudSurf.pcd'):
    path = logs / 'map' / filename
    assert path.is_file(), (filename, str(logs))
    with path.open('rb') as stream:
        for line in stream:
            if line.startswith(b'POINTS '):
                assert int(line.split()[1]) > 0, filename
                break
            assert not line.startswith(b'DATA '), 'Missing PCD point count'
print('PASS', args.vehicle, 'poses:', count, 'map:', logs / 'map', flush=True)

if rviz_shutdown_crashes:
    print('WARNING: RViz2 SIGSEGV only after shutdown; SLAM and map-save checks passed.', flush=True)
    (logs / 'rviz_shutdown_warning.txt').write_text('\n'.join(rviz_shutdown_crashes))
