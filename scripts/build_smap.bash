#!/bin/bash

set -e

declare -a TARGET_PKGS=(
        "smap"
        "smap_sampler"
        "smap_interfaces"
        "smap_classification"
        "smap_classification_wrapper"
)

#echo ${TARGET_PKGS[@]}

colcon build \
        --merge-install \
        --symlink-install \
        --packages-select ${TARGET_PKGS[@]} \
        --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
source install/setup.bash
. install/setup.bash