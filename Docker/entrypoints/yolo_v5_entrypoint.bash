#!/bin/bash
echo "---<torch_entrypoint.bash>---"
set -e

cd $WORKSPACE

# Downloading repos
(
        rm -r -f build install log | sed 's/^/  /'
        cd $WORKSPACE/src/smap/smap_yolo_v5
        vcs import < smap_yolo_v5.repos
        cd yolov5

        # File
        find  . -maxdepth 1 -type f ! \( -ipath '*.md' -o -ipath '*.git*' -o -ipath '*grep*' -o -ipath '*ignore' -o -name '.' -o -name '.pre-commit-config.yaml' -o -name '__init__.py' \) > .gitignore_f
        while read file; do (echo "${file}" | cut -c 3-) >> .gitignore_f_; done < .gitignore_f
        while read file; do (cp $file ../$file); done < .gitignore_f_

        # Dir
        find  . -maxdepth 1 -type d ! \( -ipath '*.md' -o -ipath '*.git*' -o -ipath '*grep*' -o -ipath '*ignore' -o -name '.' \) > .gitignore_d
        #while read folder; do (cut -c 3-) >> .gitignore_d_; done < .gitignore_d
        cut -c 3- .gitignore_d > .gitignore_d_
        while read folder; do (cp -r --parents $folder ./..); done < .gitignore_d_
        while read folder; do (echo "${folder}/*") >> .gitignore_d__; done < .gitignore_d_

        # Merging .gitignore
        printf "\n" >> .gitignore
        echo "## ultralytics/yolov5" >> .gitignore
        printf "\n" >> .gitignore
        echo "# Files" >> .gitignore
        while read in; do ( echo ${in}\ ) >> .gitignore; done < .gitignore_f_
        printf "\n" >> .gitignore
        echo "# Folders" >> .gitignore
        while read in; do ( echo ${in}\ ) >> .gitignore; done < .gitignore_d__

        # Should be executed just once
        #while read in; do (grep -qaexF $in ../.gitignore || echo ${in}\ ) >> ../.gitignore; done < .gitignore

        rm -fr $WORKSPACE/src/smap/smap_yolo_v5/yolov5

        cd $WORKSPACE/src/smap/smap_yolo_v5

        pip install -qr requirements.txt
        rm -fr $WORKSPACE/src/smap/smap_yolo_v5/weights
        mkdir weights
        cd $WORKSPACE/src/smap/smap_yolo_v5/weights
        python3 ../export.py --weights yolov5s.pt --include torchscript --device 0 --inplace --imgsz 640 --data ../data/coco128.yaml

        cd $WORKSPACE
)
# setup ros2 environment
(
        ./scripts/setup.bash | sed 's/^/  /' && ./scripts/full_build.bash | sed 's/^/  /'
)

. install/setup.bash

# Initial Log
echo "smap torch docker image" | sed 's/^/  /'
echo "------------------------------" | sed 's/^/  /'

echo "Image: " | sed 's/^/  /' | sed 's/^/  /'
echo "Base Image: " $IMAGE_NAME | sed 's/^/  /' | sed 's/^/  /'
echo "Ubuntu Version: " $UBUNTU_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "CUDA: " $CUDA_VERSION | sed 's/^/  /' | sed 's/^/  /'
echo "OpenGL: " $GL_VERSION | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ROS:" | sed 's/^/  /' | sed 's/^/  /'
echo "Distro: " $ROS_DISTRO | sed 's/^/  /' | sed 's/^/  /'
echo "Domain ID: " $ROS_DOMAIN_ID | sed 's/^/  /' | sed 's/^/  /'
echo "DDS middleware: " $RMW_IMPLEMENTATION | sed 's/^/  /' | sed 's/^/  /'
echo "WORKSPACE: " $WORKSPACE | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "ML: " | sed 's/^/  /'

echo "PyTorch: " $PYTORCH | sed 's/^/  /' | sed 's/^/  /'
echo "Torch CUDA: " $TORCH_CUDA | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "Torch Vision: " $TORCH_VISION | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'
echo "Torch Audio: " $TORCH_AUDIO | sed 's/^/  /' | sed 's/^/  /'  | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "Entrypoint: " | sed 's/^/  /'
echo "Host Path: " $ENTRYPOINT_HOST_PATH | sed 's/^/  /' | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "------------------------------" | sed 's/^/  /'

echo "smap pkgs:" | sed 's/^/  /'
ros2 pkg list | grep smap | sed 's/^/  /' | sed 's/^/  /'
echo "------------------------------"  | sed 's/^/  /'
echo "---</torch_entrypoint.bash>---"
exec "$@"