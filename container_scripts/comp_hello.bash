#!/bin/bash

docker build --rm -f "Docker/temp.dockerfile" -t Hello "Doc"

#docker run -it \
#           --name zed-camera-foxy \
#           --gpus all  \
#           --runtime nvidia \
#           --privileged \
#           -e DISPLAY \
#           -v /tmp/.X11-unix:/tmp/.X11-unix \
#           -w /root/ros2_ws \
#           --network host \
#           --rm \
#           zed:foxy \
#           bash -c bash -c " /root/ros2_ws && . install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=7 && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=7 ros2 launch zed_wrapper zed2.launch.py publish_tf:=false"
#