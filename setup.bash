#!/bin/bash
set -e

sudo rm -r src/turtlebot3
vcs import < src/ros2.repos src
sudo sed -i '2,5d' src/turtlebot3/turtlebot3.repos
vcs import < src/turtlebot3/turtlebot3.repos src
vcs import < src/turtlebot3/turtlebot3_ci.repos src
vcs import < src/smap/smap.repos src/smap


# vcs import < turtlebot3.repos

# Setup turtlebot3
# cd src/turtlebot3
# cd turtlebot3
# echo Builing turtlebot3
# BUILD_TYPE=RelWithDebInfo

# cd ..

# Setup ZED
# cd zed_wrapper
# ls
# echo Builing zed_wrapper
# cd ../../

source /opt/ros/foxy/setup.bash
# source install/setup.sh

# BUILD_TYPE=RelWithDebInfo
# colcon build \
#         --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
#         -Wall -Wextra -Wpedantic
# . install/setup.bash

# --merge-install \
# --symlink-install \

# sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

# source /opt/ros/foxy/setup.bash
# cd /

