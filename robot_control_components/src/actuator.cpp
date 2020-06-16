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

#include "robot_control_components/actuator.hpp"

namespace robot_control_components
{

components_ret_t Actuator::configure(ComponentInfo actuator_info)
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::init()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::recover()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::start()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::stop()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::read()
{
  components_ret_t ret = ROS2C_RETURN_OK;
  if (!can_read_) {
    ret = ROS2C_RETURN_ACTUATOR_CAN_NOT_READ;
  }
  return ret;
}

components_ret_t Actuator::write()
{
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::get_data(control_msgs::msg::InterfaceValue * data)
{
  // TODO: Add check is data are plausible
  *data = data_;
  return ROS2C_RETURN_OK;
}

components_ret_t Actuator::set_data(control_msgs::msg::InterfaceValue * data, std::string claimer_id)
{
  components_ret_t ret = ROS2C_RETURN_OK;
  if (!claimer_id_.compare(claimer_id)) {
    data_ = *data;
  }
  else {
    ret = ROS2C_RETURN_ACTUATOR_NON_CLAIMED_WRITE;
  }
  return ret;
}

std::vector<std::string> Actuator::get_interface_names()
{
  return interface_names_;
}

components_ret_t Actuator::claim(const std::string claimer_id)
{
  components_ret_t ret = ROS2C_RETURN_OK;
  if (claimer_id_.empty()) {
    claimer_id_ = claimer_id;
  }
  else {
    ret = ROS2C_RETURN_ACTUATOR_ALREADY_CLAIMED;
  }
  return ret;

}

components_ret_t Actuator::unclaim(const std::string claimer_id)
{
  components_ret_t ret = ROS2C_RETURN_OK;
  if (claimer_id_.empty()) {
    ret = ROS2C_RETURN_ACTUATOR_NOT_CLAIMED;
  }
  else if (!claimer_id_.compare(claimer_id)) {
    claimer_id_ = "";
  }
  else {
    ret = ROS2C_RETURN_ACTUATOR_UNATHORIZED_UNCLAIM;
  }
  return ret;
}

bool Actuator::isClaimed()
{
  return claimer_id_.empty();
}

bool Actuator::canRead()
{
  return can_read_;
}

}  // namespace robot_control_components
