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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/executor.hpp"

TEST(TestLoadSimpleJointLimiter, load_limiter)
{
  rclcpp::init(0, nullptr);

  using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointLimits>;
  pluginlib::ClassLoader<JointLimiter> joint_limiter_loader(
    "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>");

  std::unique_ptr<JointLimiter> joint_limiter;
  std::string joint_limiter_type = "ruckig_joint_limiter/RuckigJointLimiter";

  joint_limiter =
    std::unique_ptr<JointLimiter>(joint_limiter_loader.createUnmanagedInstance(joint_limiter_type));
  ASSERT_NO_THROW(
    joint_limiter = std::unique_ptr<JointLimiter>(
      joint_limiter_loader.createUnmanagedInstance(joint_limiter_type)));
  ASSERT_NE(joint_limiter, nullptr);
}
