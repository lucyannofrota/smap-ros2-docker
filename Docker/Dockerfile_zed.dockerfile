
# Base Image
ARG IMAGE_NAME=nvidia/cuda:11.4.1-devel-ubuntu20.04
ARG UBUNTU_VERSION=20.04
ARG CUDA_VERSION=11.4.1

FROM ${IMAGE_NAME} as zed

# Base Image
ENV IMAGE_NAME=${IMAGE_NAME}
ENV UBUNTU_VERSION=${UBUNTU_VERSION}
ENV CUDA_VERSION=${CUDA_VERSION}

ARG UBUNTU_RELEASE_YEAR=22
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=7
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

# ROS
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=${ROS_DISTRO}
ARG WORKSPACE=/workspace
ENV WORKSPACE=${WORKSPACE}
ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ARG ROS_DOMAIN_ID=7
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Entrypoint
ARG ENTRYPOINT_HOST_PATH=Docker/entrypoints/zed_entrypoint.bash
ENV ENTRYPOINT_HOST_PATH=${ENTRYPOINT_HOST_PATH}

#ARG ROS_DISTRO=humble       # ROS2 distribution

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
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-ament-cmake-clang-format \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-${RMW_IMPLEMENTATION_INSTALL} \ 
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  python3-vcstool \ 
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
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper

#ARG USERNAME=ros
#ARG USER_UID=1000
#ARG USER_GID=$USER_UID
#
## Create a non-root user
#RUN groupadd --gid $USER_GID $USERNAME \
#  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#  # [Optional] Add sudo support for the non-root user
#  && apt-get update \
#  && apt-get install -y sudo \
#  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#  && chmod 0440 /etc/sudoers.d/$USERNAME \
#  # Cleanup
#  && rm -rf /var/lib/apt/lists/* \
#  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
#  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \ 
#  && mkdir ${WORKSPACE} && chown -R ${USERNAME} ${WORKSPACE} && chown 777 -R ${WORKSPACE}

WORKDIR ${WORKSPACE}/src
# TODO: Change to vcs repos
#&& git clone --recursive https://github.com/lucyannofrota/smap_sampler.git
#&& git clone --recursive https://github.com/lucyannofrota/smap_interfaces.git
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
# TODO: Change zed wrapper to vcs repos. Use jetson version

WORKDIR ${WORKSPACE}

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "


#USER ${USERNAME}

FROM zed as setup


RUN mkdir $WORKSPACE/config
RUN cp $WORKSPACE/src/zed-ros2-wrapper/zed_wrapper/config/common.yaml $WORKSPACE/config/
RUN mv $WORKSPACE/config/common.yaml $WORKSPACE/config/p3dx.yaml
RUN sed -i 's/set_as_static: false/set_as_static: true/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/two_d_mode: false/two_d_mode: true/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/odometry_frame: "odom"/odometry_frame: "odom_zed2"/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/quality: 1/quality: 2/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/HD720/VGA/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/MEDIUM/VGA/g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/grab_frame_rate: 15/grab_frame_rate: 10 /g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/pub_frame_rate: 15.0 /pub_frame_rate: 10.0 /g' $WORKSPACE/config/p3dx.yaml && \
    sed -i 's/point_cloud_freq: 15.0 /point_cloud_freq: 10.0 /g' $WORKSPACE/config/p3dx.yaml



COPY /scripts ${WORKSPACE}/scripts
COPY ${ENTRYPOINT_HOST_PATH} /sbin/entrypoint.bash


ENTRYPOINT ["/sbin/entrypoint.bash"]
CMD ["bash"]
