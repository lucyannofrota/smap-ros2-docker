#!/bin/bash

set -e

PKG_LIST=$(ls src)

BLACKLIST=()
BLACKLIST+=("ros2.repos")
#utils
BLACKLIST+=("utils")
BLACKLIST+=("dynamixel_sdk")
BLACKLIST+=("dynamixel_sdk_custom_interfaces")
BLACKLIST+=("hls_lfcd_lds_driver")
BLACKLIST+=("dynamixel_sdk_examples")
#turtlebot3
BLACKLIST+=("turtlebot3")
BLACKLIST+=("turtlebot3_node")
BLACKLIST+=("turtlebot3_msgs")
BLACKLIST+=("turtlebot3_teleop")
BLACKLIST+=("turtlebot3_bringup")
BLACKLIST+=("turtlebot3_example")
BLACKLIST+=("turtlebot3_fake_node")
BLACKLIST+=("turtlebot3_description")
BLACKLIST+=("turtlebot3_navigation2")
BLACKLIST+=("turtlebot3_cartographer")
#zed
BLACKLIST+=("zed_interfaces")
BLACKLIST+=("zed-ros2-interfaces")

#misc
BLACKLIST+=("pkg1")
BLACKLIST+=("pkg2")


# Filter
PKG_LIST=$(ls src)

for del in ${BLACKLIST[@]}
do
        PKG_LIST=("${PKG_LIST[@]/$del}")
done


colcon build \
        --merge-install \
        --symlink-install \
        --packages-select ${PKG_LIST[@]} \
        --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
source install/setup.bash
. install/setup.bash