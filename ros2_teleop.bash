#!/bin/bash

docker exec -it p3dx bash -c "
    . /workspace/foxy/install/setup.bash &&
    ros2 run key_teleop key_teleop --ros-args \
    -p forward_rate:=0.25 -p backward_rate:=0.25 \
    -p rotation_rate:=0.7 -r /key_vel:=/cmd_vel
"