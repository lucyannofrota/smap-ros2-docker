ARG IMAGE_NAME=dustynv/ros:foxy-ros-base-l4t-r35.1.0

FROM ${IMAGE_NAME} as base_foxy

ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8
ARG JETPACK_MAJOR=5
ARG JETPACK_MINOR=0
ARG L4T_MAJOR=35
ARG L4T_MINOR=1

ARG ROS2_DIST=foxy       # ROS2 distribution

# ZED ROS2 Wrapper dependencies version
ARG XACRO_VERSION=2.0.6
ARG DIAGNOSTICS_VERSION=2.0.9
ARG AMENT_LINT_VERSION=0.12.4

ENV DEBIAN_FRONTEND noninteractive

# Enable required NVIDIA drivers
#ENV NVIDIA_DRIVER_CAPABILITIES \
#  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

# Install the ZED SDK
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO ${ROS2_DIST}

# Install the ZED ROS2 Wrapper
WORKDIR /root/ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# Install missing dependencies
WORKDIR /root/ros2_ws/src
RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
  wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
  wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint
  

FROM base_foxy as extra-pkgs

# Check that all the dependencies are satisfied
WORKDIR /root/ros2_ws
RUN apt-get update -y || true && rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/*

# Install Ping
RUN sudo apt-get update -y && sudo apt install iputils-ping

# Cyclone already set on dustynv/ros:humble-ros-base-l4t-r35.1.0

# Change ROS DDS
#ARG DEBIAN_FRONTEND=noninteractive
#RUN sudo apt-get update -y && sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

FROM extra-pkgs as configs

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build the dependencies and the ZED ROS2 Wrapper
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

WORKDIR /root/ros2_ws

# Setup environment variables
#COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
#RUN sudo chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT []
#CMD ["bash"]


