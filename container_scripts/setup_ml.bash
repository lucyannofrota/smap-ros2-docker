#!/bin/bash

cd dusty-nv

sudo ./scripts/docker_build_ros.sh -distro foxy --package desktop

#sudo ./dusty-nv/scripts/docker_run.sh --container nvcr.io/nvidia/l4t-ml:r35.2.1-py3 --volume /container_files/ml:/application

cd ..