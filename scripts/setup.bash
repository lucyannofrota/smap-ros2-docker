#!/bin/bash
set -e

git config --global --add safe.directory '*'

sudo rm -r -f build install log
sudo rm -r -f src/turtlebot3
vcs import < src/ros2.repos src
sudo sed -i '2,5d' src/turtlebot3/turtlebot3.repos
vcs import < src/turtlebot3/turtlebot3.repos src
vcs import < src/turtlebot3/turtlebot3_ci.repos src
# TODO: Make the repositories public
#vcs import < src/smap/smap.repos src/smap




source /opt/ros/foxy/setup.bash

sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y