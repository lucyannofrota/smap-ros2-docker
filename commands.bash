#!/bin/bash

sudo ls

docker-compose up -d noetic p3dx_noetic noetic_foxy p3dx_noetic_foxy 2>&1 | tee p3dx.txt
# docker-compose up -d p3dx_noetic_foxy_cont 2>&1 | tee p3dx.txt

currenttime=$(date +%H:%M)
echo $currenttime

if [[ "$currenttime" > "20:00" ]] && [[ "$currenttime" < "08:00" ]]; then
    sudo shutdown now
else
    sudo shutdown 20:00
fi
