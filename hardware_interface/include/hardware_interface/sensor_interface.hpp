// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_

#include "hardware_interface/hardware_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hardware_interface
{
/// Virtual Class to implement when integrating a stand-alone sensor into ros2_control.
/**
  * The typical examples are Force-Torque Sensor (FTS), Interial Measurement Unit (IMU).
  */
class SensorInterface : public ReadOnlyHardwareInterface
{
public:
  SensorInterface() : ReadOnlyHardwareInterface() {}

  virtual ~SensorInterface() = default;

  using ReadOnlyHardwareInterface::ReadOnlyHardwareInterface;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
