#!/bin/bash

# Carrega o ambiente do workspace
source /opt/ros/noetic/setup.bash
source "$(dirname "$0")/devel/setup.bash"

# Conecta ao ROS Master do Docker
export ROS_MASTER_URI=http://localhost:11311

# IP do computador (ajuste se necessário)
export ROS_IP=$(hostname -I | awk '{print $1}')
# ou use:
# export ROS_HOSTNAME=$(hostname)

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"

exec bash