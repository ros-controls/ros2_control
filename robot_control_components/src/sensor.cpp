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

#include "robot_control_components/sensor.hpp"

namespace robot_control_components
{

components_ret_t Sensor::configure(ComponentInfo sensor_info)
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::init()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::recover()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::start()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::stop()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::read()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Sensor::get_value(control_msgs::msg::InterfaceValue * data)
{
  // TODO: Add check is data are plausible
  *data = data_;
  return ROS2C_RETURN_OK;
}

std::vector<std::string> Sensor::get_interface_names()
{
  return interface_names_;
};

}  // namespace robot_control_components
