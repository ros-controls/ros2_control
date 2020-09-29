// Copyright 2020 ros2_control Development Team
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

#include <memory>
#include <string>
#include <vector>

#include "./test_joint_component.hpp"

using hardware_interface::return_type;

return_type
TestJointComponent::configure(
  const hardware_interface::components::ComponentInfo & /* joint_info */)
{
  return return_type::OK;
}

std::vector<std::string>
TestJointComponent::get_command_interfaces() const
{
  return {"test_command_interface"};
}

std::vector<std::string>
TestJointComponent::get_state_interfaces() const
{
  return {"test_state_interface"};
}

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(TestJointComponent, hardware_interface::components::Joint)
