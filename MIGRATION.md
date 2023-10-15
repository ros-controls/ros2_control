# Migration Notes

API changes in ros2_control releases

## ROS Rolling

#### Controller Interface

`update_reference_from_subscribers()` method got parameters and now has the following signature:
```
update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
```
