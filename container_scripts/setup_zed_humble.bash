#!/bin/bash

docker build --pull --rm -f "Docker/Dockerfile_zed_humble.dockerfile" -t zed:humble "Docker"

docker run -it \
           --name zed-camera-humble \
           --rm \
           --gpus all  \
           --runtime nvidia \
           --privileged \
           -e DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -w /root/ros2_ws \
           zed:humble \
           bash -c ". install/setup.bash ; RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=7 ros2 launch zed_wrapper zed2.launch.py"

#cd ros2_ws
#. install/setup.bash
#ROS_DOMAIN_ID=7 ros2 launch zed_wrapper zed2.launch.py
#. install/setup.bash && ROS_DOMAIN_ID=7 ros2 launch zed_wrapper zed2.launch.py
#ROS_DOMAIN_ID=7 ros2 topic list