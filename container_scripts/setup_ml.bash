#!/bin/bash

cd dusty-nv

sudo ./scripts/docker_build_ros.sh --distro foxy --package desktop

sudo ./scripts/docker_run.sh --container ros:foxy-desktop-l4t-r35.2.1 --volume /container_files/ml:/application

cd ..