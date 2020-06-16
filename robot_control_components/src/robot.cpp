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


#include "rclcpp/rclcpp.hpp"

#include "robot_control_components/robot.hpp"
#include "robot_control_components/component_parser.hpp"

namespace robot_control_components
{

components_ret_t Robot::configure(const std::string & urdf_string)
{
  //TODO: Add catch
  try {
    ComponentInfo robot_info = parse_robot_from_urdf(urdf_string);
  }
  catch (const std::runtime_error& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ros2_control_Robot"), "Error while parsing URDF: " + std::string(e.what()));
    return ROS2C_RETURN_ERROR;
  }



  return ROS2C_RETURN_OK;
}

}  // namespace robot_control_components
