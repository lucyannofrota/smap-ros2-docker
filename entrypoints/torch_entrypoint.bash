#!/bin/bash
echo "---<torch_entrypoint.bash>---"
set -e

# Downloading repos
(
        rm -r -f build install log | sed 's/^/  /'
)

# setup ros2 environment
(
        ./setup.bash | sed 's/^/  /' && ./full_build.bash | sed 's/^/  /'
)
#exec bash
. install/setup.bash

# Initial Log
echo "smap torch docker image" | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "CUDA: " $CUDA_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " $ROS_DISTRO | sed 's/^/  /' | sed 's/^/  /'
echo "Domain ID: " $ROS_DOMAIN_ID | sed 's/^/  /' | sed 's/^/  /'
echo "DDS middleware: " $RMW_IMPLEMENTATION | sed 's/^/  /' | sed 's/^/  /'
echo "Workspace: " $WORKSPACE | sed 's/^/  /'
echo "ML: " | sed 's/^/  /'
echo "PyTorch: " $TORCH_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "Torch Vision: " $TORCH_VISION_VERSION | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "Torch Audio: " $TORCH_AUDIO_VERSION | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'
echo "smap pkgs:" | sed 's/^/  /'
ros2 pkg list | grep smap | sed 's/^/  /' | sed 's/^/  /'
echo "------------------------------"  | sed 's/^/  /'
echo "---</env_entrypoint.bash>---"
exec "$@"