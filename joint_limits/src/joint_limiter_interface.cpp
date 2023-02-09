// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

/// \author Denis Stogl

#include "joint_limits/joint_limiter_interface.hpp"

#include <string>
#include <vector>

#include "joint_limits/joint_limits_rosparam.hpp"

// TODO(anyone): Add handing of SoftLimits
namespace joint_limits
{
template <>
bool JointLimiterInterface<JointLimits>::init(
  const std::vector<std::string> joint_names, const rclcpp::Node::SharedPtr & node,
  const std::string & /*robot_description_topic*/)
{
  number_of_joints_ = joint_names.size();
  joint_names_ = joint_names;
  joint_limits_.resize(number_of_joints_);
  node_ = node;

  bool result = true;

  // TODO(destogl): get limits from URDF

  // Initialize and get joint limits from parameter server
  for (auto i = 0ul; i < number_of_joints_; ++i)
  {
    if (!declare_parameters(joint_names[i], node))
    {
      RCLCPP_ERROR(
        node->get_logger(), "JointLimiter: Joint '%s': parameter declaration has failed",
        joint_names[i].c_str());
      result = false;
      break;
    }
    if (!joint_limits::get_joint_limits(joint_names[i], node, joint_limits_[i]))
    {
      RCLCPP_ERROR(
        node->get_logger(), "JointLimiter: Joint '%s': getting parameters has failed",
        joint_names[i].c_str());
      result = false;
      break;
    }
    RCLCPP_INFO(
      node->get_logger(), "Joint '%s':\n%s", joint_names[i].c_str(),
      joint_limits_[i].to_string().c_str());
  }

  if (result)
  {
    result = on_init();
  }

  return result;
}

}  // namespace joint_limits
