#!/bin/bash
echo "---<full_build.bash>---"
set -e

source /opt/ros/foxy/setup.bash

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-clean-cache \
        --continue-on-error \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic | sed 's/^/  /'
source install/setup.bash | sed 's/^/  /'
. install/setup.bash | sed 's/^/  /'
echo "---</full_build.bash>---"

# --allow-overriding slam_toolbox \