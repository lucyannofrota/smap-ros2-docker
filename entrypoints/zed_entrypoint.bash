#!/bin/bash
set -e

# setup ros2 environment
source ~/.bashrc
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$WORKSPACE/install/local_setup.bash"

# Initial Log
echo "ZED2 Camera Docker Image"
echo "------------------------------"
echo "Ubuntu Version: " $UBUNTU_RELEASE_YEAR
echo "CUDA: " $CUDA_VERSION
echo "ROS:"
echo "  Distro: " $ROS_DISTRO 
echo "  Domain ID: " $ROS_DOMAIN_ID
echo "  DDS middleware: " $RMW_IMPLEMENTATION
echo "------------------------------"  
echo "Zed pkgs"
ros2 pkg list | grep zed
echo "------------------------------"  
exec "$@"