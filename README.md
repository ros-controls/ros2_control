# ros2_control

## CI
linux: [![Build Status](https://travis-ci.org/ros-controls/ros2_control.svg?branch=crystal)](https://travis-ci.org/ros-controls/ros2_control)

## Disclaimer

`ros2_control` is a proof of concept on how new features within ROS 2 can be elaborated and used in the context of robot control.
We were keeping the vocabulary close the ROS 1 implemenation, however this is with no notion a migration, but rather a fresh new-write.
We hope that this can be a starting point for migrating ros_control to ROS 2 at some point.
Reasons for re-writing this repo from scratch rather than porting existing code to ROS 2 syntax is to leverage the new ROS 2 concepts in full.
The work done in this repo (together with [`ros2_controllers`](https://github.com/ros-controls/ros2_controllers)) was presented at [ROSCon 2017](https://vimeo.com/236182180) and is currently not under active development.

## Getting Started

In order to be able to compile these two repos, a complete ROS 2 installation is necessary.
Please find instructions on how to install ROS 2 [here](https://github.com/ros2/ros2/wiki/Installation).
At the time of writing, there exist binaries for ROS 2 (Bouncy Bolson) for all three major operating systems as well as detailed installation instructions when compiling from source.

Once ROS 2 is successfully installed, an overlay workspace can be created for the ros2_control packages.

``` bash
$ mkdir -p ~/ros2_control_ws/src
$ cd $_
$ git clone https://github.com/ros-controls/ros2_control.git
$ git clone https://github.com/ros-controls/ros2_controlllers.git
```

We can then compile the overlay workspace. For this we first have to source the ROS 2 installation. In this case, we source the `setup.bash` from the Bouncy binary distribution for Ubuntu. Obviously, the path to the setup file is different on each platform and thus has to be adjusted. Once ROS 2 is sourced, we can compile the ros2_control packages.

``` bash
$ cd ~/ros2_control_ws
$ source /opt/ros/bouncy/setup.bash # this has to be adjusted for ROS-Distro and/or OS
$ colcon build
```

## Controller Architecture

There are currently two controller available:
* JointStateController
* JointTrajectoryController

Both can be loaded through the controller manager from the [`ament_resource_index`](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md).

What's new in ROS 2 are [Managed Nodes](https://github.com/ros2/ros2/wiki/Managed-Nodes).
That means, every `LifecycleNode` adheres to an underlying state machine as described in the linked wiki page.
We feature this for the [controller interface](https://github.com/ros-controls/ros2_control/blob/crystal/controller_interface/include/controller_interface/controller_interface.hpp) in this work and thus every controller implicitely has means to start and stop the controller.
Similar to the ROS 1 implementation, each controller has to implement two functions: `init` and `update`.
We refer to the [ROS 1 wiki](http://wiki.ros.org/ros_control) for a general overview.

## Writing a demo for your own robot

Similar as to the ROS 1 implementation, every hardware platform has to implement the [`robot_hardware_interface`](https://github.com/ros-controls/ros2_control/blob/crystal/hardware_interface/include/hardware_interface/robot_hardware_interface.hpp)
Essentially, these three functions are required for every hardware and have to be implemented in a hardware-dependent manner.

``` c++
  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t init() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t read() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t write() = 0;
```

One possible example could be:

``` c++
hardware_interface::hardware_interface_ret_t
MyRobot::init()
{
  auto joint_names = {
    "my_robot_joint_1",
    "my_robot_joint_2",
  };
  size_t i = 0;
  for (auto & joint_name : joint_names) {
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    joint_state_handles_[i] = state_handle;
    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::HW_RET_OK) {
      throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
    }

    hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    joint_command_handles_[i] = command_handle;
    if (register_joint_command_handle(&joint_command_handles_[i]) !=
      hardware_interface::HW_RET_OK)
    {
      throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    }
    ++i;
  }
}

hardware_interface::hardware_interface_ret_t
MyRobot::read()
{
  // do robot specific stuff to update the pos_, vel_, eff_ arrays
}

hardware_interface::hardware_interface_ret_t
MyRobot::write()
{
  // do robot specific stuff to apply the command values from cmd_ to the robot
}
```

This robot hardware can then be loaded through the controller manager:

``` c++
void
spin(std::shared_ptr<rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main()
{
  // do all the init stuff
  
  // create my_robot instance
  auto my_robot = std::make_shared<MyRobot>();
  
  // initialize the robot
  if (my_robot->init() != hardware_interface::HW_RET_OK) {
    fprintf(stderr, "failed to initialized yumi hardware\n");
    return -1;
  }
  
  auto executor =
    std::make_shared<rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor>();
  
  // start the controller manager with the robot hardware
  controller_manager::ControllerManager cm(my_robot, executor);
  // load the joint state controller.
  // "ros_controllers" is the resource index from where to look for controllers
  // "ros_controllers::JointStateController" is the class we want to load
  // "my_robot_joint_state_controller" is the name for the node to spawn
  cm.load_controller(
    "ros_controllers",
    "ros_controllers::JointStateController",
    "my_robot_joint_state_controller");
  // load the trajectory controller
  cm.load_controller(
    "ros_controllers",
    "ros_controllers::JointTrajectoryController",
    "my_robot_joint_trajectory_controller");

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    RCUTILS_LOG_ERROR("at least one controller failed to configure")
    return -1;
  }
  // and activate all controller
  if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    RCUTILS_LOG_ERROR("at least one controller failed to activate")
    return -1;
  }

  // main loop
  hardware_interface::hardware_interface_ret_t ret;
  while (active) {
    ret = my_robot->read();
    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "read failed!\n");
    }

    cm.update();

    ret = my_robot->write();
    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "write failed!\n");
    }

    r.sleep();
  }

  executor->cancel();
}
```
