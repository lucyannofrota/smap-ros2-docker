#!/bin/bash

# sudo ls

cd p3dx-docker

compose_push () {
    # $1 service
    # $2 tag_name
    # $3 idx
    # (docker-compose up -d $1 && docker tag $2 docker.io/lucyannofrota/$2) 2>&1 | tee stp$3.txt    
    (docker-compose up -d $1 && docker tag $2 docker.io/lucyannofrota/$2 && docker image push docker.io/lucyannofrota/$2) 2>&1 | tee stp$3.log    
}

compose_push noetic jetson:noetic 1
compose_push noetic_foxy jetson:noetic-foxy 2

compose_push noetic_p3dx p3dx:noetic 3
compose_push noetic_foxy_p3dx p3dx:noetic-foxy 4


# (docker-compose up -d noetic jetson_noetic && docker image push lucyannofrota/p3dx:noetic) 2>&1 | tee stp1.txt



# (docker-compose up -d a2 && docker tag p3dx:a2 docker.io/lucyannofrota/p3dx:a2 && docker image push docker.io/lucyannofrota/p3dx:a2) 2>&1 | tee stp1.txt

# docker image push lucyannofrota/p3dx:noetic 2>>&1 | tee stp1.txt

#  noetic_foxy p3dx_noetic p3dx_noetic_foxy p3dx_noetic_pkgs p3dx_noetic_foxy_pkgs 2>&1 | tee stp1.txt

# docker image push lucyannofrota/p3dx:t

# docker-compose up -d jetson_noetic jetson_noetic_foxy jetson_noetic_foxy_bridge 2>&1 | tee jetson.txt

# currenttime=$(date +%H:%M)
# echo $currenttime

# if [[ "$currenttime" > "20:00" ]] && [[ "$currenttime" < "08:00" ]]; then
#     sudo shutdown now
# else
#     sudo shutdown 20:00
# fi
