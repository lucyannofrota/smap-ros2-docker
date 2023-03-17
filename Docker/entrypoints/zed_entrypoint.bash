#!/bin/bash
echo "---<zed_entrypoint.bash>---"
set -e

cd $WORKSPACE

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$WORKSPACE/install/local_setup.bash" --

# Welcome information
echo "ZED ROS2 Docker Image"
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

echo 'Available ZED packages:'
ros2 pkg list | grep zed
echo "------------------------------" | sed 's/^/  /'
echo "---</zed_entrypoint.bash>---"
exec "$@"