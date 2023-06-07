// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_SIMPLE_JOINT_LIMITER_HPP_
#define TEST_SIMPLE_JOINT_LIMITER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

const double COMMON_THRESHOLD = 0.0011;

#define CHECK_STATE_SINGLE_JOINT(tested_traj_point, idx, expected_pos, expected_vel, expected_acc) \
  EXPECT_NEAR(tested_traj_point.positions[idx], expected_pos, COMMON_THRESHOLD);                   \
  EXPECT_NEAR(tested_traj_point.velocities[idx], expected_vel, COMMON_THRESHOLD);                  \
  EXPECT_NEAR(tested_traj_point.accelerations[idx], expected_acc, COMMON_THRESHOLD);

#define INTEGRATE(cur, des, dt)                                                      \
  cur.positions[0] += des.velocities[0] * dt + 0.5 * des.accelerations[0] * dt * dt; \
  cur.velocities[0] += des.accelerations[0] * dt;

using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointLimits>;

class SimpleJointLimiterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    auto testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    node_ = std::make_shared<rclcpp::Node>(testname);
  }

  SimpleJointLimiterTest()
  : joint_limiter_loader_(
      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>")
  {
    joint_limiter_type_ = "joint_limits/SimpleJointLimiter";
  }

  void TearDown() override { node_.reset(); }

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<JointLimiter> joint_limiter_;
  std::string joint_limiter_type_;
  pluginlib::ClassLoader<JointLimiter> joint_limiter_loader_;

  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
  trajectory_msgs::msg::JointTrajectoryPoint desired_joint_states_;
  trajectory_msgs::msg::JointTrajectoryPoint current_joint_states_;
};

#endif  // TEST_SIMPLE_JOINT_LIMITER_HPP_
