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

#ifndef HARDWARE_INTERFACE__ACTUATOR_HANDLE_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_HANDLE_HPP_

#include <string>

#include "hardware_interface/handle.hpp"

namespace hardware_interface
{
/** A handle used to get and set a value on a given actuator interface. */
class ActuatorHandle : public ReadWriteHandle<ActuatorHandle>
{
public:
  using ReadWriteHandle<ActuatorHandle>::ReadWriteHandle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ACTUATOR_HANDLE_HPP_
