#!/bin/bash

# sudo ./build_images.bash 2>&1 | tee p3dx-docker/logs/master_log.log

exec 3>&1 1> >(tee /mnt/chopin_02/p3dx-docker/logs/master_log.log) 2>&1

set -e

sudo ls > /dev/null

cd p3dx-docker

printf "Building images\nStart: " > logs/build.log

date "+%H:%M:%S   %d/%m/%y" >> logs/build.log
SECONDS=0
# start=$(date +%s)

printf "\n\n" >> logs/build.log



compose () {
    # $1 service
    # $2 tag_name
    # $3 idx
    # (docker-compose up -d $1 && docker tag $2 docker.io/lucyannofrota/$2) 2>&1 | tee stp$3.txt    
    # (docker-compose build --no-cache $1 && docker tag $2 docker.io/lucyannofrota/$2 && docker image push docker.io/lucyannofrota/$2) 2>&1 | tee stp$3.log    
    printf "\t\tBuilding $1\n\t\t\tStart: " >> logs/build.log
    date "+%H:%M:%S   %d/%m/%y" >> logs/build.log
    start_var=$(date +%s)
    # ((docker-compose build --no-cache $1) && (docker tag $2 $3) && (docker push $3)) 2>&1 | tee logs/$1.log    
    ((docker-compose build --no-cache $1) && (docker tag $2 $3)) 2>&1 | tee logs/$1.log    

    printf "\t\t\tEnd:   " >> logs/build.log
    date "+%H:%M:%S   %d/%m/%y" >> logs/build.log
    end_var=$(date +%s)
    printf "\t\t\tElapsed Time: $(($end_var-$start_var)) seconds\n\n" >> logs/build.log
}

# printf "\tP3-DX:\n\n" >> logs/build.log

# compose noetic build:jetson-noetic lucyannofrota/jetson-noetic:alpha
# compose noetic_foxy build:jetson-noetic-foxy lucyannofrota/jetson-noetic-foxy:alpha
# compose noetic_p3dx build:p3dx-noetic lucyannofrota/p3dx-noetic:alpha
# compose noetic_foxy_p3dx build:p3dx-noetic-foxy lucyannofrota/p3dx-noetic-foxy:alpha
# compose p3dx_navigation build:p3dx-navigation lucyannofrota/p3dx-navigation:alpha

# printf "\tSMAP:\n\n" >> logs/build.log

# compose ros2-zed build:zed lucyannofrota/foxy:zed-alpha
# compose smap-sampler-zed build:jetson-sampler-zed lucyannofrota/smap:jetson-sampler-zed-alpha
# compose smap-yolov5 build:jetson-yolov5 lucyannofrota/smap:jetson-yolov5-alpha
# compose smap-env build:jetson-env lucyannofrota/smap:jetson-env-alpha
compose smap-deploy build:jetson-deploy lucyannofrota/smap:jetson-deploy-alpha



printf "\n\nEnd:   " >> logs/build.log

date "+%H:%M:%S   %d/%m/%y" >> logs/build.log
end=$(date +%s)
echo "Elapsed Time: $(($SECONDS)) seconds" >> logs/build.log

# shutdown +30


# currenttime=$(date +%H:%M)
# echo $currenttime

# if [[ "$currenttime" > "20:00" ]] && [[ "$currenttime" < "08:00" ]]; then
#     sudo shutdown now
# else
#     sudo shutdown 20:00
# fi
