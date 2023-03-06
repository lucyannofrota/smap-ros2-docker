#!/bin/bash

set -e

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-clean-cache \
        --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
source install/setup.bash
. install/setup.bash