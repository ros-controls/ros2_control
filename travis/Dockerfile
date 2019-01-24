FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    bash-completion \
    dirmngr \
    git \
    gnupg2 \
    libasio-dev \
    libtinyxml2-dev \
    lsb-release \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install packages from the ROS repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-pkg-modules \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# clone source
ENV ROS2_WS /root/ros2_ws
RUN mkdir -p $ROS2_WS/src
COPY ./workspace.repos $ROS2_WS
WORKDIR $ROS2_WS
RUN vcs import src < workspace.repos

ENV ROS_DISTRO crystal
# install dependencies
RUN apt-get update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# build source
WORKDIR $ROS2_WS
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && colcon \
    build \
    --cmake-args -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install \
    --event-handler console_direct+"]
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source $ROS2_WS/install/setup.bash && colcon \
    test \
    --event-handler console_direct+"]
RUN colcon test-result
