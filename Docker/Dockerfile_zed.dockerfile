# Container based on https://github.com/stereolabs/zed-ros2-wrapper/blob/master/docker/Dockerfile.u22-cu117-humble-devel

#ARG IMAGE_NAME=nvcr.io/nvidia/cuda:11.4.1-devel-ubuntu20.04
ARG IMAGE_NAME=nvidia/cuda:11.4.1-devel-ubuntu20.04

FROM ${IMAGE_NAME}

ARG UBUNTU_RELEASE_YEAR=20
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=4
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

ARG ROS2_DIST=foxy       # ROS2 distribution

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

############ Install ROS2 ############
ENV ROS_DISTRO ${ROS2_DIST}

# Set and Check Locale
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8 && \
  locale  # verify settings && \
  rm -rf /var/lib/apt/lists/*

# Setup Sources
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y software-properties-common && \
  add-apt-repository universe && \
  apt-get install -y curl && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  rm -rf /var/lib/apt/lists/*

# Install ROS 2 Base packages and Python dependencies
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y \
  ros-${ROS2_DIST}-ros-base \
  ros-${ROS2_DIST}-ament-cmake-clang-format \
  ros-${ROS2_DIST}-image-transport \
  ros-${ROS2_DIST}-image-transport-plugins \
  ros-${ROS2_DIST}-diagnostic-updater \
  ros-${ROS2_DIST}-xacro \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools && \
  pip3 install \
  argcomplete \
  numpy \
  empy \
  lark && \
  rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install the ZED SDK
RUN echo "CUDA Version $CUDA_VERSION" > /usr/local/cuda/version.txt

# Setup the ZED SDK
RUN apt-get update -y || true && \
  apt-get install --no-install-recommends dialog bash-completion lsb-release wget less udev sudo  build-essential cmake zstd python3 python3-pip libpng-dev libgomp1 -y && \
  python3 -m pip install --upgrade pip; python3 -m pip install numpy opencv-python-headless && \
  wget -q -O ZED_SDK_Linux_Ubuntu.run https://download.stereolabs.com/zedsdk/$ZED_SDK_MAJOR.$ZED_SDK_MINOR/cu$CUDA_MAJOR$CUDA_MINOR/ubuntu$UBUNTU_RELEASE_YEAR && \
  chmod +x ZED_SDK_Linux_Ubuntu.run && \
  ./ZED_SDK_Linux_Ubuntu.run -- silent skip_tools skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm ZED_SDK_Linux_Ubuntu.run && \
  rm -rf /var/lib/apt/lists/* && \
  apt-get autoremove && apt-get clean

# Install the ZED ROS2 Wrapper
RUN mkdir /workspaces && mkdir /workspaces/zed && mkdir /workspaces/zed/src
WORKDIR /workspaces/zed/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
WORKDIR /workspaces/zed

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "


# Change ROS FOXY DDS
ARG DEBIAN_FRONTEND=noninteractive
RUN sudo apt-get update -y && sudo apt install -y ros-foxy-rmw-cyclonedds-cpp

#CMD ["bash"]
ENTRYPOINT []
#entrypoint: [ "bash", "-c", "ros2 launch zed_wrapper zed2.launch.py" ]

#CMD ["ros2 launch zed_wrapper zed2.launch.py"]