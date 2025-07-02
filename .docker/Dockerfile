ARG ROS_DISTRO=jazzy
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

FROM ros:${ROS_DISTRO} AS base
LABEL maintainer="Sebastian Jahr sebastian.jahr@tuta.io"

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION

ENV DEBIAN_FRONTEND=noninteractive
ENV USERNAME=maurizio
ENV WORKSPACE=/home/${USERNAME}/workspace

# Create user and workspace directory
RUN useradd -m -u 1010 -s /bin/bash ${USERNAME} && \
    mkdir -p ${WORKSPACE} && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

COPY . ${WORKSPACE}/src/krakel

# Install Pinocchio
RUN apt update && \
    apt upgrade -y && \
    apt install -y ros-${ROS_DISTRO}-pinocchio

# Set up the entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN echo "source /sbin/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
