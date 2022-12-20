##############################################
# Created from https://github.com/athackst/dockerfiles/blob/main/ros2/foxy-cuda.Dockerfile
##############################################
# FROM althack/ros2:foxy-cuda-full
FROM althack/ros2:foxy-cuda-gazebo-nvidia as base

# ** [Optional] Uncomment this section to install additional packages. **
#

# Turtlebot3
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
# 	&& apt-get -y install --no-install-recommends \
# 	#
# 	# Clean up
# 	#    && apt-get autoremove -y \
# 	#    && apt-get clean -y \
# 	#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc


FROM base as turtlebot


# Turtlebot3
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros

RUN apt-get update \
	&& apt-get -y install --no-install-recommends \
	ros-foxy-gazebo-* \
	ros-foxy-cartographer \
	ros-foxy-cartographer-ros \
	ros-foxy-navigation2 \
	ros-foxy-nav2-bringup \
	ros-foxy-dynamixel-sdk \
	ros-foxy-turtlebot3* \
	&& echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc \
	&& echo "export export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc

	#
	# Clean up
	#    && apt-get autoremove -y \
	#    && apt-get clean -y \
	#    && rm -rf /var/lib/apt/lists/*

# SHELL ["echo", "export ROS_DOMAIN_ID=30 #TURTLEBOT3", ">> ~/.bashrc"]
# SHELL ["/bin/bash", "-c", "source", "~/.bashrc"]

ENV DEBIAN_FRONTEND=dialog


FROM turtlebot as test_turtlebot

# RUN /workspaces/Semantic-Mapping-ROS2-Containers/setup.sh


# RUN apt-get update \
# 	&& apt-get -y install --no-install-recommends \
# 	&& echo "export export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashr

# ENV DEBIAN_FRONTEND=dialog

# FROM turtlebot as testing

# RUN echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/$USERNAME/.bashrc
# RUN echo "ls" >> /home/$USERNAME/.bashrc