// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class RobotHardwareInterface
{
public:
  HARDWARE_INTERFACE_PUBLIC
  RobotHardwareInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~RobotHardwareInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type init() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type read() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type write() = 0;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_
