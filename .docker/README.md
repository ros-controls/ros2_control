# ROS2 Control Docker Containers

Meant to provide basic docker containers for CI or development purposes. To use them, make sure you have [Docker](https://docs.docker.com/get-docker/) installed. You then can pull the latest source or build image or build any version and run the image by following tag specific commands as described below:

## Source folder and tag
Downloads and builds ros2_controls into a workspace for development use exactly as is found [here](https://control.ros.org/master/doc/getting_started/getting_started.html#building-from-source). This is primarily used for development and CI of ros2_control and related packages.

You can pull a prebuilt image with this docker tag: `ghcr.io/ros-controls/ros2_control_source`.

To build and run the container, execute the following code in a terminal:
```
docker build --tag ros2_control:source --file .docker/source/Dockerfile .
docker run -it ros2_control:source
```

If you are not familiar with docker, you'll want to consider how you want to maintain persistent data across container sessions. The above commands with start a **new** container and that container will persist as long as you don't remove it. If you want to return to it, you'll have to use the `docker start` command with the container id.

A better way to maintain your code changes across coding sessions is to use docker volumes or bind mounts. Web search those terms for more documentation but a simple example is provided below. First, you must build the container as shown in the first command above, then:

Bind mount example:
```
docker run -it --mount type=bind,source=/local/path/to/repo,destination=/root/ws_ros2_control/src/<path to repo you want replaced> ros2_control:source
```
Note: the above example replaces the containers contents with whatever you mount.

Volume example:
```
docker run -it --mount type=volume,source=name-of-volume,destination=/root/ws_ros2_control/src/ ros2_control:source
```
Note: this is probably the preferred solution as changes persist across rebuilds of container and doesn't require you to pull all repositories that you want to edit as they'll already be in the container.

## Release folder and tag
Installs ros2_control in the base ros2 docker container as mentioned [here](https://control.ros.org/master/doc/getting_started/getting_started.html#building-from-source). This is mainly intended for development of external packages that rely on ros2_control.

You can pull a prebuilt image using the following tag: `ghcr.io/ros2-controls/ros2_control_release`.

It can be built and run by the following commands in the directory of the ros2_control repository:
```
docker build --tag ros2_control:release --file .docker/release/Dockerfile .
docker run -it ros2_control:release
```
