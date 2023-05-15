#!/bin/bash

docker exec -it p3dx bash -c "
    . /workspace/foxy/install/setup.bash &&
    ros2 run key_teleop key_teleop -r /key_vel:=/cmd_vel
"