#!/bin/bash

# Script used to setup the docker path location
# https://linuxconfig.org/how-to-move-docker-s-default-var-lib-docker-to-another-directory-on-ubuntu-debian-linux
# Most of the tutorial above is valid, but the '-g' flag is outdated; it was replaced by '--data-root'! (https://docs.docker.com/engine/reference/commandline/dockerd/)

sudo systemctl stop docker.service
sudo systemctl stop docker.socket

sudo rm /lib/systemd/system/docker.service
sudo cp /mnt/chopin_02/docker.service /lib/systemd/system/docker.service

sudo systemctl daemon-reload
sudo systemctl start docker
