// Copyright (c) 2021, PickNik, Inc.
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

#include <vector>

#include "hardware_interface/handle.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rcppmath/clamp.hpp"

// Floating-point value comparison threshold
const double EPS = 1e-12;

TEST(SaturateTest, Saturate)
{
  const double min = -1.0;
  const double max = 2.0;
  double val;

  val = -0.5;
  EXPECT_NEAR(val, rcppmath::clamp(val, min, max), EPS);

  val = 0.5;
  EXPECT_NEAR(val, rcppmath::clamp(val, min, max), EPS);

  val = -1.0;
  EXPECT_NEAR(val, rcppmath::clamp(min, min, max), EPS);

  val = -2.0;
  EXPECT_NEAR(min, rcppmath::clamp(val, min, max), EPS);

  val = 2.0;
  EXPECT_NEAR(val, rcppmath::clamp(val, min, max), EPS);

  val = 3.0;
  EXPECT_NEAR(max, rcppmath::clamp(val, min, max), EPS);
}

class TestJointLimiter : public joint_limits::JointLimiterInterface<joint_limits::JointLimits>
{
public:
  TestJointLimiter(
    joint_limits::JointLimits & limits,
    std::vector<hardware_interface::StateInterface> & state_interfaces,
    std::vector<hardware_interface::CommandInterface *> & command_interfaces)
  : JointLimiterInterface(limits, state_interfaces, command_interfaces)
  {}

protected:
  void enforce_effort_limits(const rclcpp::Duration & /*period*/) override
  {}
  void enforce_pos_vel_acc_limits(const rclcpp::Duration & /*period*/) override
  {}
};

class JointLimiterTest : public ::testing::Test
{
public:
  JointLimiterTest()
  {
    limits.has_position_limits = true;
    limits.min_position = -1.0;
    limits.max_position = 1.0;

    limits.has_velocity_limits = true;
    limits.max_velocity = 2.0;

    limits.has_effort_limits = true;
    limits.max_effort = 8.0;
  }

  void SetUp()
  {
    commands_storage_.resize(4, 0.0);
    states_storage_.resize(4, 0.0);
  }

  joint_limits::JointLimits limits;

  std::vector<double> commands_storage_;
  std::vector<double> states_storage_;
};

TEST_F(JointLimiterTest, Construction)
{
  // Test throw on missing command interfaces
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    std::vector<hardware_interface::CommandInterface *> command_interfaces;
    EXPECT_THROW(
      TestJointLimiter(limits, state_interfaces, command_interfaces), std::runtime_error);
  }

  // Test OK when at least one command interface is provided
  {
    joint_limits::JointLimits limits_bad;
    std::vector<hardware_interface::StateInterface> state_interfaces;
    std::vector<hardware_interface::CommandInterface *> command_interfaces;
    EXPECT_THROW(
      TestJointLimiter(limits_bad, state_interfaces, command_interfaces), std::runtime_error);
  }

  // Test bad limits
//   {
//     joint_limits::JointLimits limits_bad;
//     std::vector<hardware_interface::StateInterface> state_interfaces;
//     std::vector<hardware_interface::CommandInterface *> command_interfaces;
//     EXPECT_THROW(
//       TestJointLimiter(limits_bad, state_interfaces, command_interfaces), std::runtime_error);
//   }
}
