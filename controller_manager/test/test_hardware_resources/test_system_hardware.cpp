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

#include "./test_system_hardware.hpp"

using hardware_interface::hardware_interface_status;
using hardware_interface::return_type;

hardware_interface::return_type
TestSystemHardware::configure(const hardware_interface::HardwareInfo & /* system_info */)
{
  return return_type::OK;
}

return_type
TestSystemHardware::start()
{
  return return_type::OK;
}

return_type
TestSystemHardware::stop()
{
  return return_type::OK;
}

hardware_interface_status
TestSystemHardware::get_status() const
{
  return hardware_interface_status::UNKNOWN;
}

return_type
TestSystemHardware::read_sensors(
  std::vector<std::shared_ptr<hardware_interface::components::Sensor>> & /* sensors */) const
{
  return return_type::OK;
}

return_type
TestSystemHardware::read_joints(
  std::vector<std::shared_ptr<hardware_interface::components::Joint>> & /* joints */) const
{
  return return_type::OK;
}

return_type
TestSystemHardware::write_joints(
  const std::vector<std::shared_ptr<hardware_interface::components::Joint>> & /* joints */)
{
  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(TestSystemHardware, hardware_interface::SystemHardwareInterface)
