#!/bin/bash

docker exec -it zed-wrapper bash -c " \
        . install/setup.bash \ 
        ros2 run smap_sampler smap_sampler_node
"