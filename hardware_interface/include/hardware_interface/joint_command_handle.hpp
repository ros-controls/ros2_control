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

#ifndef HARDWARE_INTERFACE__JOINT_COMMAND_HANDLE_HPP_
#define HARDWARE_INTERFACE__JOINT_COMMAND_HANDLE_HPP_

#include <string>

#include "hardware_interface/visibility_control.h"

#include "types/hardware_interface_return_values.hpp"

namespace hardware_interface
{
/** A handle used to get and set the command of a single joint. */
class JointCommandHandle
{
public:
  HARDWARE_INTERFACE_PUBLIC
  JointCommandHandle();

  /**
   * The commmand handles are passive in terms of ownership.
   * That means that the handles are only allowed to read/write
   * the storage, however are not allowed to delete or reallocate
   * this memory.
   * \param name The name of the joint
   * \param cmd A pointer to the storage for this joint's command
   */
  HARDWARE_INTERFACE_PUBLIC
  JointCommandHandle(
    const std::string & name,
    double * cmd);

  HARDWARE_INTERFACE_PUBLIC
  const std::string &
  get_name() const;

  HARDWARE_INTERFACE_PUBLIC
  double
  get_cmd() const;

  HARDWARE_INTERFACE_PUBLIC
  void
  set_cmd(double cmd);

  HARDWARE_INTERFACE_PUBLIC
  bool valid_pointers() const;

private:
  std::string name_;
  double * cmd_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__JOINT_COMMAND_HANDLE_HPP_
