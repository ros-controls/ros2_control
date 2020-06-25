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

#ifndef HARDWARE_INTERFACE__JOINT_STATE_HANDLE_HPP_
#define HARDWARE_INTERFACE__JOINT_STATE_HANDLE_HPP_

#include <string>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/** A handle used to read the state of a single joint. */
class JointStateHandle
{
public:
  HARDWARE_INTERFACE_PUBLIC
  JointStateHandle();

  /**
   * The joint handles are passive in terms of ownership.
   * That means that the handles are only allowed to read/write
   * the storage, however are not allowed to delete or reallocate
   * this memory.
   * \param name The name of the joint
   * \param pos A pointer to the storage for this joint's position
   * \param vel A pointer to the storage for this joint's velocity
   * \param eff A pointer to the storage for this joint's effort (force or torque)
   */
  HARDWARE_INTERFACE_PUBLIC
  JointStateHandle(
    const std::string & name,
    const double * pos,
    const double * vel,
    const double * eff);

  HARDWARE_INTERFACE_PUBLIC
  const std::string &
  get_name() const;

  HARDWARE_INTERFACE_PUBLIC
  double
  get_position() const;

  HARDWARE_INTERFACE_PUBLIC
  double
  get_velocity() const;

  HARDWARE_INTERFACE_PUBLIC
  double
  get_effort() const;

  HARDWARE_INTERFACE_PUBLIC
  bool valid_pointers() const;

private:
  std::string name_;
  const double * pos_;
  const double * vel_;
  const double * eff_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__JOINT_STATE_HANDLE_HPP_
