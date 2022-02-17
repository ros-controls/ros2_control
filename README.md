# ros2_control

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Foxy** | [`foxy`](https://github.com/ros-controls/ros2_control/tree/foxy) | [![Foxy Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> [![Foxy Source Build](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control/actions/workflows/foxy-source-build.yml?branch=foxy) | [Documentation](https://control.ros.org/) | [ros2_control](https://index.ros.org/p/ros2_control/#foxy)
**Galactic** | [`galactic`](https://github.com/ros-controls/ros2_control/tree/galactic) | [![Galactic Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> [![Galactic Source Build](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control/actions/workflows/galactic-source-build.yml?branch=galactic) | [Documentation](https://control.ros.org/) | [ros2_control](https://index.ros.org/p/ros2_control/#galactic)
**Rolling - Last Focal** | [`rolling`](https://github.com/ros-controls/ros2_control/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build-last-focal.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build-last-focal.yml?branch=master) | [Documentation](https://control.ros.org/) | [ros2_control](https://index.ros.org/p/ros2_control/#rolling)
**Rolling** | [`rolling`](https://github.com/ros-controls/ros2_control/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> [![Rolling Source Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-source-build.yml?branch=master) | [Documentation](https://control.ros.org/) | [ros2_control](https://index.ros.org/p/ros2_control/#rolling)


## About
This package is a part of the ros2_control framework.
For more, please check the [documentation](https://ros-controls.github.io/control.ros.org/).


## Docker images
There are a few published docker images that come with the latest releases. More information about them can be found in the `.docker` folder. You can pull them under these tags: `ghcr.io/ros-controls/ros2_control_release` or `ghcr.io/ros-controls/ros2_control_source`.


## Acknowledgements

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.


### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
