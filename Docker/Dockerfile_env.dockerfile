ARG IMAGE_NAME=dustynv/ros:foxy-desktop-l4t-r35.2.1

FROM dustynv/ros:foxy-desktop-l4t-r35.2.1

FROM ${IMAGE_NAME} as base

ARG IMAGE_NAME
ENV IMAGE_NAME=${IMAGE_NAME}

ARG UBUNTU_VERSION=20.04
ENV UBUNTU_VERSION=${UBUNTU_VERSION}

ARG CUDA_VERSION=11.4.2
ENV CUDA_VERSION=${CUDA_VERSION}

ARG ROS_DISTRO=noetic
ENV ROS_DISTRO=${ROS_DISTRO}

ARG WORKSPACE=/workspace
ENV WORKSPACE=${WORKSPACE}


ARG RMW_IMPLEMENTATION_INSTALL=rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION_INSTALL=${RMW_IMPLEMENTATION_INSTALL}

ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

ARG ROS_DOMAIN_ID=7
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

ARG ENTRYPOINT_HOST_PATH=entrypoints/p3dx_entrypoint.bash
ENV ENTRYPOINT_HOST_PATH=${ENTRYPOINT_HOST_PATH}

FROM base as setup


ARG ROS_DISTRO
ARG WORKSPACE
ARG ENTRYPOINT_HOST_PATH

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


#Add environment dependencies


RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}


COPY ${ENTRYPOINT_HOST_PATH} /sbin/entrypoint.bash

#COPY scripts ${WORKSPACE}/scripts
#COPY entrypoints/env.repos ${WORKSPACE}/src/env.repos

RUN chown -R ${USERNAME} ${WORKSPACE} && chmod 777 -R ${WORKSPACE} \
 && chown ${USERNAME} /sbin/entrypoint.bash && chmod 777 /sbin/entrypoint.bash

USER ${USERNAME}

ENTRYPOINT ["/sbin/entrypoint.bash"]
CMD ["bash"]
