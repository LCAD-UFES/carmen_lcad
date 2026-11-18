#!/bin/bash
# Roda um comando ROS 1 (Noetic) do workspace src/sc_lio_sam com o ambiente limpo.
#
# O ~/.bashrc do ARGOS carrega o ROS 2 Foxy, e o proccontrol herda o ambiente do terminal
# que o abriu: PYTHONPATH/AMENT_PREFIX_PATH/ROS_DISTRO do Foxy misturados com o Noetic fazem
# o roslaunch achar os pacotes errados. Aqui tudo do Foxy sai antes de carregar o Noetic.
#
# Uso (a partir de bin/, como no process):
#   ../data/argos/ros1_noetic.sh roslaunch lio_sam run.launch ...
#   ../data/argos/ros1_noetic.sh roscore

strip_foxy()
{
	echo "$1" | tr ':' '\n' | grep -v -e '/opt/ros/foxy' -e '^$' | paste -sd: -
}

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
	ROS_PACKAGE_PATH CMAKE_PREFIX_PATH RMW_IMPLEMENTATION
export PATH=$(strip_foxy "$PATH")
export PYTHONPATH=$(strip_foxy "$PYTHONPATH")
export LD_LIBRARY_PATH=$(strip_foxy "$LD_LIBRARY_PATH")

CARMEN_HOME=${CARMEN_HOME:-$(cd "$(dirname "$0")/../.." && pwd)}
export CARMEN_HOME
source /opt/ros/noetic/setup.bash
source "$CARMEN_HOME/src/sc_lio_sam/devel/setup.bash"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

exec "$@"
