#!/bin/bash
set -e
# Start ROS2 without inheriting the ROS1 workspace environment.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "$workspace/install/local_setup.bash"
exec ros2 launch carmen_ros_bridge remaining_ros1.launch.py "$@"
