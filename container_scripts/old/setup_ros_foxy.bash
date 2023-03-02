#!/bin/bash

docker run -it --name ros_foxy --runtime nvidia --gpus all --network host -v /mnt/chopin_02/container_files/ros_foxy:/applications dustynv/ros:foxy-ros-base-l4t-r35.1.0