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

#include "controller_interface/controller_interface.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
ControllerInterface::ControllerInterface() : ControllerInterfaceBase() {}

bool ControllerInterface::is_chainable() const { return false; }

std::vector<hardware_interface::CommandInterface> ControllerInterface::export_reference_interfaces()
{
  return {};
}

bool ControllerInterface::set_chained_mode(bool /*chained_mode*/) { return false; }

bool ControllerInterface::is_in_chained_mode() const { return false; }

}  // namespace controller_interface
