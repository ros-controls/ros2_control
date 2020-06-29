// Copyright 2020 ROS2-Control Development Team
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

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/component_interfaces/actuator_interface.hpp"

namespace robot_control_components
{

using hardware_interface::hardware_interface_ret_t;
using hardware_interface::ComponentInfo;

class VelocityActuator : public hardware_interface::ActuatorInterface
{
public:
  std::vector<std::string> allowed_interfce_types;

  VelocityActuator() = default;

  ~VelocityActuator() = default;

  hardware_interface_ret_t configure(const ComponentInfo & actuator_info)
  {
    (void) actuator_info;

    allowed_interfce_types.push_back(hardware_interface::HW_IF_VELOCITY);

    return hardware_interface_ret_t::HW_RET_OK;
  }

  std::string get_interface_name() const
  {
    return "";
  }

  hardware_interface_ret_t start()
  {
    return hardware_interface_ret_t::HW_RET_OK;
  }

  hardware_interface_ret_t stop()
  {
    return hardware_interface_ret_t::HW_RET_OK;
  }

  bool is_started() const
  {
    return false;
  }

  hardware_interface_ret_t read(double & data)
  {
    (void) data;
    return hardware_interface_ret_t::HW_RET_OK;
  }

  hardware_interface_ret_t write(const double & data)
  {
    (void) data;
    return hardware_interface_ret_t::HW_RET_OK;
  }
};

}  // namespace robot_control_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_control_components::VelocityActuator, hardware_interface::ActuatorInterface)
