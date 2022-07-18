robot agnostic hardware_interface package.
This package will eventually be moved into its own repo.

## Best practice for `hardware_interface` implementation
In the following section you can find some advices which will help you implement interface
for your specific hardware.

### Best practice for having different update rate for each `hardware_interface` by counting loops
Current implementation of [ros2_control main node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp)
has one update rate that controls the rate of the [`read()`](https://github.com/ros-controls/ros2_control/blob/fe462926416d527d1da163bc3eabd02ee1de9be9/hardware_interface/include/hardware_interface/system_interface.hpp#L169) and [`write()`](https://github.com/ros-controls/ros2_control/blob/fe462926416d527d1da163bc3eabd02ee1de9be9/hardware_interface/include/hardware_interface/system_interface.hpp#L178)
calls in [`hardware_interface(s)`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp).
In this section suggestion on how to run each interface implementation on its own update rate is introduced.

1. Add parameters of main control loop update rate and desired update rate for your hardware interface.
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="system_interface" params="name main_loop_update_rate desired_hw_update_rate">

    <ros2_control name="${name}" type="system">
      <hardware>
          <plugin>my_system_interface/MySystemHardware</plugin>
          <param name="main_loop_update_rate">${main_loop_update_rate}</param>
          <param name="desired_hw_update_rate">${desired_hw_update_rate}</param>
      </hardware>
      ...
    </ros2_control>

  </xacro:macro>

</robot>
```

2. In you [`on_init()`](https://github.com/ros-controls/ros2_control/blob/fe462926416d527d1da163bc3eabd02ee1de9be9/hardware_interface/include/hardware_interface/system_interface.hpp#L94) specific implementation fetch desired parameters:
```
namespace my_system_interface
{
hardware_interface::CallbackReturn MySystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //   declaration in *.hpp file --> unsigned int main_loop_update_rate_, desired_hw_update_rate_ = 100 ;
  main_loop_update_rate_ = stoi(info_.hardware_parameters["main_loop_update_rate"]);
  desired_hw_update_rate_ = stoi(info_.hardware_parameters["desired_hw_update_rate"]);

  ...
}
...
} // my_system_interface
```

3. In your `on_activate()` specific implementation reset internal loop counter
```
hardware_interface::CallbackReturn MySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    //   declaration in *.hpp file --> unsigned int update_loop_counter_ ;
    update_loop_counter_ = 0;
    ...
}
```

4. In your `read(const rclcpp::Time & time, const rclcpp::Duration & period)` and/or
   `write(const rclcpp::Time & time, const rclcpp::Duration & period)`
   specific implementations decide if you should interfere with your hardware
```
hardware_interface::return_type MySystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  bool hardware_go = main_loop_update_rate_ == 0  ||
                     desired_hw_update_rate_ == 0 ||
                     ((update_loop_counter_ % desired_hw_update_rate_) == 0);

  if (hardware_go){
    // hardware comms and operations

  }
  ...

  // update counter
  ++update_loop_counter_;
  update_loop_counter_ %= main_loop_update_rate_;
}
```

### Best practice for having different update rate for each `hardware_interface` by measuring elapsed time
Another way to decide if hardware communication should be executed in the`read(const rclcpp::Time & time, const rclcpp::Duration & period)` and/or
`write(const rclcpp::Time & time, const rclcpp::Duration & period)` implementations is to measure elapsed time since last pass:

```
hardware_interface::CallbackReturn MySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    //   declaration in *.hpp file --> bool first_read_pass_, first_write_pass_ = true ;
    first_read_pass_ = first_write_pass_ = true;
    ...
}

hardware_interface::return_type MySystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (first_read_pass_ || (time - last_read_time_ ) > period)
    {
      first_read_pass_ = false;
      //   declaration in *.hpp file --> rclcpp::Time last_read_time_ ;
      last_read_time_ = time;
      // hardware comms and operations
      ...
    }
    ...
}

hardware_interface::return_type MySystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (first_write_pass_ || (time - last_write_time_ ) > period)
    {
      first_write_pass_ = false;
      //   declaration in *.hpp file --> rclcpp::Time last_write_time_ ;
      last_write_time_ = time;
      // hardware comms and operations
      ...
    }
    ...
}
```
