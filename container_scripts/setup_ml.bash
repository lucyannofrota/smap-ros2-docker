#!/bin/bash

docker run -it --name ml --runtime nvidia --gpus all --network host -v /mnt/chopin_02/container_files/ml:/applications nvcr.io/nvidia/l4t-ml:r35.1.0-py3