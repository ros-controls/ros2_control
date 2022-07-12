ARG ROS_DISTRO="rolling"
FROM ros:$ROS_DISTRO
ARG BRANCH="master"

ENV ROS_UNDERLAY /root/ws_ros2_control/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/ros-controls/ros2_control/$BRANCH/ros2_control.$ROS_DISTRO.repos ros2_control.repos
RUN vcs import < ros2_control.repos

RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

RUN cd $ROS_UNDERLAY/.. && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY/setup.bash"' \
      /ros_entrypoint.sh
