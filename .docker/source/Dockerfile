ARG ROS_DISTRO="galactic"
FROM ros:${ROS_DISTRO}

ENV ROS_UNDERLAY /root/ws_ros2_control/install
WORKDIR $ROS_UNDERLAY/../src

RUN apt-get update && apt-get install wget && \
        wget https://raw.githubusercontent.com/ros-controls/ros2_control/master/ros2_control/ros2_control.repos && \
        vcs import < ros2_control.repos

RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/

RUN cd $ROS_UNDERLAY/.. && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY/setup.bash"' \
      /ros_entrypoint.sh
