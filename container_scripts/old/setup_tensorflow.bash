#!/bin/bash

docker run -it --name tensorflow-1.15 --runtime nvidia --gpus all --network host -v /mnt/chopin_02/container_files/tensorflow:/applications nvcr.io/nvidia/l4t-tensorflow:r35.1.0-tf1.15-py3