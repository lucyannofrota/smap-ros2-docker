#!/bin/bash
set -e

vcs import < src/ros2.repos src

# Setup turtlebot3
# cd src/turtlebot3
# vcs import < turtlebot3.repos
# cd turtlebot3
# echo Builing turtlebot3
# BUILD_TYPE=RelWithDebInfo

# cd ..

# Setup ZED
# cd zed_wrapper
# ls
# echo Builing zed_wrapper
# cd ../../

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
. install/setup.bash

sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
# cd /

