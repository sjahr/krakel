ARG ROS_DISTRO=rolling
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

FROM moveit/moveit2:${ROS_DISTRO}-source
LABEL maintainer="Sebastian Jahr sebastian.jahr@tuta.io"

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=${ROS_UNDERLAY}/..

COPY . ${WORKSPACE}/src/krakel

# Install pre-commit
RUN apt-get update && \
    apt-get install --no-install-recommends \
        ca-certificates \
        gnupg \
        lsb-release \
        python3-pip \
        wget -y && \
    pip3 install --break-system-packages pre-commit

# Install ROS 2 dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE}/src && \
    vcs import < krakel/.docker/upstream.repos --recursive

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    apt-get update && \
    apt upgrade -y && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    . install/setup.sh && \
    nice colcon build --mixin release

# Set up the entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN echo "source /sbin/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
