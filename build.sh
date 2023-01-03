#!/bin/bash
set -e

# Set the default build type
# BUILD_TYPE=RelWithDebInfo
BUILD_TYPE=None
sudo colcon build \
        --merge-install \
        --packages-select zed_components \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic

source install/setup.bash

# --packages-select semantic_mapping \