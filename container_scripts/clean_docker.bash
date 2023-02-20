#!/bin/bash

docker ps -aq | xargs docker stop | xargs docker rm
docker system prune -f
docker system prune -a -f