// Copyright 2020 ros2_control development team
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

#ifndef TRANSMISSION_INTERFACE__HANDLE_HPP_
#define TRANSMISSION_INTERFACE__HANDLE_HPP_

#include "hardware_interface/handle.hpp"

namespace transmission_interface
{
/** A handle used to get and set a value on a given actuator interface. */
class ActuatorHandle : public hardware_interface::Handle
{
public:
  using hardware_interface::Handle::Handle;
};

/** A handle used to get and set a value on a given joint interface. */
class JointHandle : public hardware_interface::Handle
{
public:
  using hardware_interface::Handle::Handle;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__HANDLE_HPP_
