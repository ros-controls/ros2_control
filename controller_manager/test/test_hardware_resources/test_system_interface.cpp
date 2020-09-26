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
#include <vector>

#include "./test_system_interface.hpp"

hardware_interface::return_type
TestSystem::configure(const hardware_interface::HardwareInfo & system_info)
{
  fprintf(stderr, "configuring plugin with name %s\n", system_info.name.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestSystem::start()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestSystem::stop()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::hardware_interface_status
TestSystem::get_status() const
{
  return hardware_interface::hardware_interface_status::UNKNOWN;
}

hardware_interface::return_type
TestSystem::read_sensors(
  std::vector<std::shared_ptr<hardware_interface::components::Sensor>> & sensors) const
{
  (void) sensors;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestSystem::read_joints(
  std::vector<std::shared_ptr<hardware_interface::components::Joint>> & joints) const
{
  (void) joints;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestSystem::write_joints(
  const std::vector<std::shared_ptr<hardware_interface::components::Joint>> & joints)
{
  (void) joints;
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(TestSystem, hardware_interface::SystemHardwareInterface)
