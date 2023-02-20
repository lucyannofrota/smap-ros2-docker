#!/bin/bash

# docker run -it --runtime nvidia --gpus all --network host -v mnt/chopin_02/container_files/ml nvcr.io/nvidia/l4t-ml:r35.1.0-py3

docker run -it --name pytorch-1.12 --runtime nvidia --gpus all --network host -v /mnt/chopin_02/container_files/pytorch:/applications nvcr.io/nvidia/l4t-pytorch:r35.1.0-pth1.12-py3

#docker run --gpus all -it nvcr.io/nvidia/pytorch:23.01-py3