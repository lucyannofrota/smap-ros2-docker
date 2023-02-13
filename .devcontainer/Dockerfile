##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image 
###########################################
FROM nvidia/cuda:11.7.0-devel-ubuntu20.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

###########################################
#  Develop image 
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  # Install ros distro testing packages
  ros-foxy-ament-lint \
  ros-foxy-launch-testing \
  ros-foxy-launch-testing-ament-cmake \
  ros-foxy-launch-testing-ros \
  python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Full image 
###########################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y \
  ros-foxy-desktop \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

###########################################
#  Full+Gazebo image 
###########################################
FROM full AS gazebo

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN apt-get update && apt-get install -q -y \
  lsb-release \
  wget \
  gnupg \
  sudo \
  && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y \
    ros-foxy-gazebo* \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

###########################################
#  Full+Gazebo+Nvidia image 
###########################################

FROM gazebo AS gazebo-nvidia

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1




###########################################
#  Full+Gazebo+Nvidia+ZED-sdk image 
###########################################

FROM gazebo-nvidia as zed-sdk

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility,graphics

RUN echo "Europe/Paris" > /etc/localtime ; echo "CUDA Version 11.7" > /usr/local/cuda/version.txt

USER ros
ENV USER=ros
# Install zstd
RUN sudo apt-get update -y || true
RUN sudo apt-get install --no-install-recommends zstd

# Setup the ZED SDK
RUN sudo apt-get update -y || true
RUN sudo apt-get install --no-install-recommends lsb-release wget less udev sudo build-essential cmake -y
RUN sudo wget -q -O ZED_SDK_38_Linux_Ubuntu20.run https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu20
RUN sudo chmod +x ZED_SDK_38_Linux_Ubuntu20.run

RUN ./ZED_SDK_38_Linux_Ubuntu20.run -- silent

RUN sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so \ 
    && sudo rm ZED_SDK_38_Linux_Ubuntu20.run \
    && sudo rm -rf /var/lib/apt/lists/*

FROM zed-sdk as zed-api

# USER ros
# ENV USER=ros

# # ZED Python API
RUN sudo apt-get update -y || true
RUN sudo apt-get install --no-install-recommends python3 python3-pip libpng-dev libgomp1 -y && \ 
    wget download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py && \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install numpy opencv-python pyopengl *.whl && \
    sudo rm *.whl ; sudo rm -rf /var/lib/apt/lists/*

# # Make some tools happy
RUN sudo mkdir -p /root/Documents/ZED/

# ROS diagnostic_updater
RUN sudo apt-get update -y || true
RUN sudo apt-get install --no-install-recommends \
    ros-foxy-diagnostic-updater

##########################################
#  Full+Gazebo+Nvidia+ZED-sdk+turtlebot image 
##########################################

FROM zed-api as turtlebot

# Turtlebot3
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
# Set up auto-source of workspace for ros user
ARG WORKSPACE="/workspaces/Semantic-Mapping-ROS2-Containers"
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# RUN sudo apt-get update \
# 	&& sudo apt-get -y install --no-install-recommends \
# 	ros-foxy-gazebo-* \
# 	ros-foxy-cartographer \
# 	ros-foxy-cartographer-ros \
# 	ros-foxy-navigation2 \
# 	ros-foxy-nav2-bringup \
# 	ros-foxy-dynamixel-sdk \
# 	ros-foxy-turtlebot3* \
#   ros-foxy-pluginlib \
# 	&& echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc \
# 	&& echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc \
# 	&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/Semantic-Mapping-ROS2-Containers/src/turtlebot3/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
# 	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

RUN sudo apt-get update
RUN sudo apt-get -y install --no-install-recommends \
  ros-foxy-gazebo-* \
	ros-foxy-cartographer \
	ros-foxy-cartographer-ros \
	ros-foxy-navigation2 \
	ros-foxy-nav2-bringup \
	ros-foxy-dynamixel-sdk \
	ros-foxy-turtlebot3*
RUN echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc \
	&& echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc \
	&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/Semantic-Mapping-ROS2-Containers/src/turtlebot3/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1"  >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=dialog