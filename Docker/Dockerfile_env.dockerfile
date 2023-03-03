##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image 
###########################################
#FROM nvidia/cuda:11.4.1-devel-ubuntu20.04 AS base
#FROM nvidia/cuda:11.7.0-devel-ubuntu20.04 AS base
FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04 AS base

ENV ROS_DISTRO=foxy

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
    ros-${ROS_DISTRO}-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
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
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
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
  ros-${ROS_DISTRO}-desktop \
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
    ros-${ROS_DISTRO}-gazebo* \
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
##########################################
#  Full+Gazebo+Nvidia+ZED-sdk+turtlebot image 
##########################################

FROM gazebo-nvidia as turtlebot

# Turtlebot3
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
# Set up auto-source of workspace for ros user
ARG WORKSPACE="/workspaces/smap-ros2-containers"
#RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# RUN sudo apt-get update \
# 	&& sudo apt-get -y install --no-install-recommends \
# 	ros-${ROS_DISTRO}-gazebo-* \
# 	ros-${ROS_DISTRO}-cartographer \
# 	ros-${ROS_DISTRO}-cartographer-ros \
# 	ros-${ROS_DISTRO}-navigation2 \
# 	ros-${ROS_DISTRO}-nav2-bringup \
# 	ros-${ROS_DISTRO}-dynamixel-sdk \
# 	ros-${ROS_DISTRO}-turtlebot3* \
#   ros-${ROS_DISTRO}-pluginlib \
# 	&& echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc \
# 	&& echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc \
# 	&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/smap-ros2-containers/src/turtlebot3/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
# 	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
# 	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

RUN sudo apt-get update
RUN sudo apt-get -y install --no-install-recommends \
  ros-${ROS_DISTRO}-gazebo-* \
	ros-${ROS_DISTRO}-cartographer \
	ros-${ROS_DISTRO}-cartographer-ros \
	ros-${ROS_DISTRO}-navigation2 \
	ros-${ROS_DISTRO}-nav2-bringup \
	ros-${ROS_DISTRO}-dynamixel-sdk \
	ros-${ROS_DISTRO}-turtlebot3*
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc \
	&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/smap-ros2-containers/src/turtlebot3/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc \


#echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc \
ENV DEBIAN_FRONTEND=dialog

FROM turtlebot as env_setup

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install DDS
ARG DEBIAN_FRONTEND=noninteractive
RUN sudo apt-get update -y && sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}


RUN echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1"  >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
  && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

#ENTRYPOINT [  ]
#CMD ["sleep", "infinity"]
#RUN cd ${WORKSPACE}
#RUN cd ${WORKSPACE} && sudo ./setup.bash
#RUN sudo ./build.bash