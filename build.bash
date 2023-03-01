#!/bin/bash

set -e

# Set the default build type
# BUILD_TYPE=RelWithDebInfo


# source /opt/ros/foxy/setup.bash

#BUILD_TYPE=RelWithDebInfo
#--cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
source install/setup.sh

# --packages-select semantic_mapping \

# source install/setup.bash