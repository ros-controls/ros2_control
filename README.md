# ros2_control

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/ros2_control/branch/humble/graph/badge.svg?token=idvm1zJXOf)](https://codecov.io/gh/ros-controls/ros2_control/tree/humble)

This package is a part of the ros2_control framework.
For more, please check the [documentation](https://control.ros.org/).


## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_control/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/rolling-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html) | [ros2_control](https://index.ros.org/p/ros2_control/#rolling)
**Jazzy** | [`master`](https://github.com/ros-controls/ros2_control/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/jazzy-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/jazzy-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/jazzy-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/jazzy/index.html) <br /> [API Reference](https://control.ros.org/jazzy/doc/api/index.html) | [ros2_control](https://index.ros.org/p/ros2_control/#jazzy)
**Iron** | [`iron`](https://github.com/ros-controls/ros2_control/tree/iron) | [![Iron Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/iron-binary-build.yml/badge.svg?branch=iron)](https://github.com/ros-controls/ros2_control/actions/workflows/iron-binary-build.yml?branch=iron) <br /> [![Iron Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/iron-semi-binary-build.yml/badge.svg?branch=iron)](https://github.com/ros-controls/ros2_control/actions/workflows/iron-semi-binary-build.yml?branch=iron) | [Documentation](https://control.ros.org/iron/index.html) <br /> [API Reference](https://control.ros.org/iron/doc/api/index.html) | [ros2_control](https://index.ros.org/p/ros2_control/#iron)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_control/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/humble-binary-build.yml?branch=master) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_control/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control/actions/workflows/humble-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/humble/index.html) <br /> [API Reference](https://control.ros.org/humble/doc/api/index.html) | [ros2_control](https://index.ros.org/p/ros2_control/#humble)

[Detailed build status](.github/workflows/README.md)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


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
