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

#include <string>

#include "hardware_interface/component_interfaces/actuator_interface.hpp"

namespace robot_control_components
{

namespace hw = hardware_interface;

class PositionActuator : public hardware_interface::ActuatorInterface
{
public:
  hw::return_type configure(const hw::ComponentInfo & /* actuator_info */)
  {
    return hw::return_type::OK;
  }

  std::string get_interface_name() const
  {
    return "";
  }

  hw::return_type start()
  {
    return hw::return_type::OK;
  }

  hw::return_type stop()
  {
    return hw::return_type::OK;
  }

  bool is_started() const
  {
    return false;
  }

  hw::return_type read(double & /* data */)
  {
    return hw::return_type::OK;
  }

  hw::return_type write(const double & /* data */)
  {
    return hw::return_type::OK;
  }
};

}  // namespace robot_control_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_control_components::PositionActuator, hardware_interface::ActuatorInterface)
