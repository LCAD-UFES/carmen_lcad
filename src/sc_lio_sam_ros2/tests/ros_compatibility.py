#!/usr/bin/env python3
"""Check parameter and TF2 compatibility using the currently sourced ROS 2."""
import os
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parent / 'ros_compatibility'
with tempfile.TemporaryDirectory(prefix='ros-compatibility-') as temporary:
    subprocess.run(['cmake', '-S', str(source), '-B', temporary], check=True)
    subprocess.run(['cmake', '--build', temporary, '--parallel', '2'], check=True)
    environment = dict(os.environ, ROS_DOMAIN_ID='166', ROS_LOG_DIR=temporary + '/ros')
    subprocess.run(['ctest', '--output-on-failure'], cwd=temporary, env=environment, check=True)
