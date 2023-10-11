#!/bin/bash

cd /mnt/chopin_02/datasets

clear () {
    echo "Cleaning /mnt/chopin_02/datasets"
    sudo find . -name "*.txt" -delete
    sudo find . -name "*.pgm" -delete
    sudo find . -name "*.yaml" -delete
    sudo find . -name "*.smap" -delete
    sudo find . -name "*.dot" -delete
    sudo find . -name "*.pdf" -delete
}

clear