#!/bin/bash

docker run -it \
    --name ros_foxy_desktop \
    --runtime nvidia \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v $XSOCK:$XSOCK \
    -v $HOME/.Xauthority:/root/.Xauthority \
    --privileged \
    -v /mnt/chopin_02/container_files/ros_foxy:/applications \
    dustynv/ros:foxy-desktop-l4t-r35.1.0

#--network host \