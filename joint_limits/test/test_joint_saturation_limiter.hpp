// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_JOINT_SATURATION_LIMITER_HPP_
#define TEST_JOINT_SATURATION_LIMITER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

const double COMMON_THRESHOLD = 0.0011;

#define CHECK_STATE_SINGLE_JOINT(tested_traj_point, idx, expected_pos, expected_vel, expected_acc) \
  EXPECT_NEAR(tested_traj_point.positions[idx], expected_pos, COMMON_THRESHOLD);                   \
  EXPECT_NEAR(tested_traj_point.velocities[idx], expected_vel, COMMON_THRESHOLD);                  \
  EXPECT_NEAR(tested_traj_point.accelerations[idx], expected_acc, COMMON_THRESHOLD);

using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointLimits>;

class JointSaturationLimiterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_name_ = ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  void SetupNode(const std::string node_name = "")
  {
    if (!node_name.empty())
    {
      node_name_ = node_name;
    }
    node_ = std::make_shared<rclcpp::Node>(node_name_);
  }

  void Load()
  {
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_));
  }

  void Init(const std::string & joint_name = "foo_joint")
  {
    joint_names_ = {joint_name};
    joint_limiter_->init(joint_names_, node_);
    num_joints_ = joint_names_.size();
    last_commanded_state_.positions.resize(num_joints_, 0.0);
    last_commanded_state_.velocities.resize(num_joints_, 0.0);
    last_commanded_state_.accelerations.resize(num_joints_, 0.0);
    last_commanded_state_.effort.resize(num_joints_, 0.0);
    desired_joint_states_ = last_commanded_state_;
    current_joint_states_ = last_commanded_state_;
  }

  void Configure() { joint_limiter_->configure(last_commanded_state_); }

  void Integrate(double dt)
  {
    current_joint_states_.positions[0] += desired_joint_states_.velocities[0] * dt +
                                          0.5 * desired_joint_states_.accelerations[0] * dt * dt;
    current_joint_states_.velocities[0] += desired_joint_states_.accelerations[0] * dt;
  }

  JointSaturationLimiterTest()
  : joint_limiter_type_("joint_limits/JointSaturationLimiter"),
    joint_limiter_loader_(
      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>")
  {
  }

  void TearDown() override { node_.reset(); }

protected:
  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  std::unique_ptr<JointLimiter> joint_limiter_;
  std::string joint_limiter_type_;
  pluginlib::ClassLoader<JointLimiter> joint_limiter_loader_;

  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
  trajectory_msgs::msg::JointTrajectoryPoint desired_joint_states_;
  trajectory_msgs::msg::JointTrajectoryPoint current_joint_states_;
};

#endif  // TEST_JOINT_SATURATION_LIMITER_HPP_
