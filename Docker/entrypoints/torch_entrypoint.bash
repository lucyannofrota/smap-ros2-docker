#!/bin/bash
echo "---<torch_entrypoint.bash>---"
set -e

cd $WORKSPACE

# Downloading repos
(
        rm -r -f build install log | sed 's/^/  /'
)

# setup ros2 environment
(
        ./scripts/setup.bash | sed 's/^/  /' && ./scripts/full_build.bash | sed 's/^/  /'
)

. install/setup.bash

# Initial Log
echo "smap torch docker image" | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'

echo "Image: " | sed 's/^/  /' | sed 's/^/  /'
echo "Base Image: " $IMAGE_NAME | sed 's/^/  /' | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "CUDA: " $CUDA_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "OpenGL: " $GL_VERSION | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " $ROS_DISTRO | sed 's/^/  /' | sed 's/^/  /'
echo "Domain ID: " $ROS_DOMAIN_ID | sed 's/^/  /' | sed 's/^/  /'
echo "DDS middleware: " $RMW_IMPLEMENTATION | sed 's/^/  /' | sed 's/^/  /'
echo "WORKSPACE: " $WORKSPACE | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ML: " | sed 's/^/  /'

echo "PyTorch: " $PYTORCH | sed 's/^/  /' | sed 's/^/  /'
echo "Torch CUDA: " $TORCH_CUDA | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "Torch Vision: " $TORCH_VISION | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "Torch Audio: " $TORCH_AUDIO | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "Entrypoint: " | sed 's/^/  /'
echo "Host Path: " $ENTRYPOINT_HOST_PATH | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "smap pkgs:" | sed 's/^/  /'
ros2 pkg list | grep smap | sed 's/^/  /' | sed 's/^/  /'
echo "------------------------------"  | sed 's/^/  /'
echo "---</env_entrypoint.bash>---"
exec "$@"