##############################################
# Based on https://github.com/athackst/dockerfiles/blob/main/ros2/foxy-cuda.Dockerfile
##############################################
FROM althack/ros2:foxy-cuda-gazebo-nvidia as base

# ** [Optional] Uncomment this section to install additional packages. **
#

# Set up auto-source of workspace for ros user
ARG WORKSPACE="/workspaces/Semantic-Mapping-ROS2-Containers"
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
	&& echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc \
	&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/Semantic-Mapping-ROS2-Containers/src/turtlebot3/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1"  >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
	&& echo "export RCUTILS_COLORIZED_OUTPUT=1"  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number}) [{time}]\""  >> /home/$USERNAME/.bashrc \
	&& echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\""  >> /home/$USERNAME/.bashrc

	#
	# Clean up
	#    && apt-get autoremove -y \
	#    && apt-get clean -y \
	#    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

