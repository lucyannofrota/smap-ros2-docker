#!/bin/bash
echo "---<env_entrypoint.bash>---"
set -e

# Downloading repos
(
        rm -r -f build install log | sed 's/^/  /'
        rm -r -f src/turtlebot3 | sed 's/^/  /'
        vcs import < src/ros2.repos src | sed 's/^/  /'
        sed -i '2,5d' src/turtlebot3/turtlebot3.repos | sed 's/^/  /'
        vcs import < src/turtlebot3/turtlebot3.repos src | sed 's/^/  /'
        vcs import < src/turtlebot3/turtlebot3_ci.repos src | sed 's/^/  /'
)
# TODO: Make the repositories public
#vcs import < src/smap/smap.repos src/smap

# setup ros2 environment
(
        ./scripts/setup.bash | sed 's/^/  /' && ./scripts/full_build.bash | sed 's/^/  /'
)
. install/setup.bash

# Initial Log
echo "smap env docker image" | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "CUDA: " $CUDA_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " $ROS_DISTRO | sed 's/^/  /' | sed 's/^/  /'
echo "Domain ID: " $ROS_DOMAIN_ID | sed 's/^/  /' | sed 's/^/  /'
echo "DDS middleware: " $RMW_IMPLEMENTATION | sed 's/^/  /' | sed 's/^/  /'
echo "Workspace: " $WORKSPACE | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'
echo "Nav2 pkgs:" | sed 's/^/  /'
ros2 pkg list | grep -e nav2 | sed 's/^/  /' | sed 's/^/  /'
echo "Turtlebot3 pkgs:" | sed 's/^/  /'
ros2 pkg list | grep -e turtlebot | sed 's/^/  /' | sed 's/^/  /'
echo "Zed pkgs:" | sed 's/^/  /'
ros2 pkg list | grep -e zed | sed 's/^/  /' | sed 's/^/  /'
echo "Aux pkgs:" | sed 's/^/  /'
ros2 pkg list | { grep -e dynamixel ; grep -e hls_lfcd_lds_driver ; } | sed 's/^/  /' | sed 's/^/  /'
echo "smap pkgs:" | sed 's/^/  /'
ros2 pkg list | grep smap | sed 's/^/  /' | sed 's/^/  /'
echo "------------------------------"  | sed 's/^/  /'
echo "---</env_entrypoint.bash>---"
exec "$@"