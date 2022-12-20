#!/bin/bash
set -e

cd src

eval TEMP_FOLDER="_tmp"
eval n=2
eval GIT[0]="https://github.com/ROBOTIS-GIT/turtlebot3"
eval GIT[1]="https://github.com/ROBOTIS-GIT/turtlebot3_simulations"
eval BRANCH="foxy-devel"

for ((i=0; i <= ${n}-1; i++))
do

  git clone -b ${BRANCH} ${GIT[${i}]} ${TEMP_FOLDER}


  cd ${TEMP_FOLDER}

  ls

  for d in */; do
    echo Moving Folder ${d}
    sudo cp -a ${d} ../
  done

  cd ..
  sudo rm -r ${TEMP_FOLDER}
done