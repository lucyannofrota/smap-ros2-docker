#!/bin/bash
echo "---<ultralitics_yolo_v5_install.bash>---"
set -e

cd $WORKSPACE

rm -r -f build install log | sed 's/^/  /'
mkdir -p $WORKSPACE/src/smap/smap_yolo_v5
cd $WORKSPACE/src/smap/smap_yolo_v5

#TODO: Change this section when the github repos is available. (Download the file)
printf "# List of repositories to use within your workspace
# See https://github.com/dirk-thomas/vcstool

repositories:
  yolov5:
    type: git
    url: https://github.com/ultralytics/yolov5.git
    version: 78a90c9\n" > smap_yolo_v5.repos




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

cd ../

rm -fr yolov5



# pip install --force-reinstall -v --upgrade --upgrade-strategy only-if-needed "pandas==1.4"
# pip install --force-reinstall -v --upgrade --upgrade-strategy only-if-needed "numpy==1.19"
# pip install --force-reinstall -v --upgrade --upgrade-strategy only-if-needed "matplotlib==3.6"
# pip install --force-reinstall -v --upgrade --upgrade-strategy only-if-needed "scipy==1.9.3"
pip install --upgrade --upgrade-strategy only-if-needed -qr requirements.txt



# TODO: Change this section with the original file from the github repository

printf "# YOLOv5 requirements
# Usage: pip install -r requirements.txt

# Base ------------------------------------------------------------------------
gitpython>=3.1.30
matplotlib<3.7.0
numpy<1.20
opencv-python>=4.1.1
Pillow>=7.1.2
psutil  # system resources
PyYAML>=5.3.1
requests>=2.23.0
scipy>=1.4.1
thop>=0.1.1  # FLOPs computation
torch>=1.7.0  # see https://pytorch.org/get-started/locally (recommended)
torchvision>=0.8.1
tqdm>=4.64.0
# protobuf<=3.20.1  # https://github.com/ultralytics/yolov5/issues/8012

# Logging ---------------------------------------------------------------------
tensorboard>=2.4.1
# clearml>=1.2.0
# comet

# Plotting --------------------------------------------------------------------
pandas<1.5.0
seaborn>=0.11.0

# Export ----------------------------------------------------------------------
# coremltools>=6.0  # CoreML export
# onnx<1.13.0  # ONNX export
# onnx-simplifier>=0.4.1  # ONNX simplifier
# nvidia-pyindex  # TensorRT export
# nvidia-tensorrt  # TensorRT export
# scikit-learn<=1.1.2  # CoreML quantization
# tensorflow>=2.4.1  # TF exports (-cpu, -aarch64, -macos)
# tensorflowjs>=3.9.0  # TF.js export
# openvino-dev  # OpenVINO export

# Deploy ----------------------------------------------------------------------
setuptools>=65.5.1 # Snyk vulnerability fix
# tritonclient[all]~=2.24.0

# Extras ----------------------------------------------------------------------
# ipython  # interactive notebook
# mss  # screenshots
# albumentations>=1.0.3
# pycocotools>=2.0.6  # COCO mAP
# roboflow
# ultralytics  # HUB https://hub.ultralytics.com
" > cont_requirements.txt

pip install --upgrade --upgrade-strategy only-if-needed -qr cont_requirements.txt

# python3 ../export.py --weights yolov5s.pt --include torchscript engine onnx --device 0 --inplace --imgsz 640 --data ../data/coco128.yaml --opset 16

cd $WORKSPACE

echo "---</ultralitics_yolo_v5_install.bash>---"