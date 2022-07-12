ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers && \
    rm -rf /var/lib/apt/lists/*
