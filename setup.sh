#!/bin/bash
set -e

vcs import < src/ros2.repos src
# Setup turtlebot3
cd src/turtlebot3
vcs import < turtlebot3_ci.repos
vcs import < turtlebot3.repos
echo Builing turtlebot3
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
cd ../../
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
