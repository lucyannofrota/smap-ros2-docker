# File System

Jetpack 5.1 uses almost all the onboard storage of the Jetson AGX Xavier 32GB. Is recommendet to use and external drive.

# Remote Desktop

NoMachine has a good performance and can run natively on arm64 (https://downloads.nomachine.com/linux/?id=30&distro=Arm).



# Docker Containers

The general recommendation is to use the 'docker_run.sh' present inside 'dusty-nv/scripts' folder.

This script setup the container with GUI forwarding, GPU access, and user permissions.

**The build time of each container is ~2 hours!**

## Build / Test

Build and Test instructions can be found in https://github.com/dusty-nv/jetson-containers.

First execute the "Docker Default Runtime". \
**Don't skip the "Docker Default Runtime" step. It is mandatory to easily bypass gpu resources to containers!**



## Run

> `sudo ./dusty-nv/scripts/docker_run.sh --container ${container} --volume ${volume}`

--container -> specify the desired container \
> --container `nvcr.io/nvidia/l4t-##:r##.#.#-py3`

The references of the containers can be found in https://github.com/dusty-nv/jetson-containers.

--volume -> specify the folder to be attached to the container \

> --volume `/my/host/path:/my/container/path`

The volumes should be stored in the `container_files` folder. Each category of container shoud have an independent folder. (e.g. "nvcr.io/nvidia/l4t-ml:r35.2.1-py3" -> "/container_files/ml")
