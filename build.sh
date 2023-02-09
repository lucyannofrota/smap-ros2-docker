#!/bin/bash

set -e

# Set the default build type
# BUILD_TYPE=RelWithDebInfo
source /opt/ros/foxy/setup.bash

BUILD_TYPE=RelWithDebInfo
sudo colcon build \
        --merge-install \
        --packages-select semantic_mapping classifier_wrapper \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic

source install/setup.bash