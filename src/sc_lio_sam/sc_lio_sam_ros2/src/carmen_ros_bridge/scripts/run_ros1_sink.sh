#!/bin/bash
set -e
# Isolate Python and shared libraries from the ROS2 runtime.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/odometry_ros1_sink.py"
