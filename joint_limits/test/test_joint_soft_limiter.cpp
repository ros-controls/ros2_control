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

/// \author Adrià Roig Moreno

#include <cmath>
#include <limits>
#include "test_joint_limiter.hpp"

TEST_F(JointSoftLimiterTest, when_loading_limiter_plugin_expect_loaded)
{
  // Test JointSoftLimiter loading
  ASSERT_NO_THROW(
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_)));
  ASSERT_NE(joint_limiter_, nullptr);
}

// NOTE: We accept also if there is no limit defined for a joint.
TEST_F(JointSoftLimiterTest, when_joint_not_found_expect_init_fail)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());
  // initialize the limiter
  std::vector<std::string> joint_names = {"bar_joint"};
  ASSERT_TRUE(joint_limiter_->init(joint_names, node_));
}

TEST_F(JointSoftLimiterTest, when_invalid_dt_expect_enforce_fail)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  ASSERT_TRUE(JointLimiterTest::Init());
  ASSERT_TRUE(Configure());
  rclcpp::Duration period(0, 0);  // 0 second
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
}

TEST_F(JointSoftLimiterTest, check_desired_position_only_cases)
{
  SetupNode("soft_joint_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -M_PI;
  limits.max_position = M_PI;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  // no size check occurs (yet) so expect true
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};

  rclcpp::Duration period(1, 0);  // 1 second
  desired_state_.position = 4.0;
  EXPECT_TRUE(desired_state_.has_position());
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // For hard limits, if there is no actual state but the desired state is outside the limit, then
  // saturate it to the limits
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), limits.max_position, COMMON_THRESHOLD);
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  desired_state_.position = -5.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), limits.min_position, COMMON_THRESHOLD);
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // If the desired state is within the limits, then no saturation is needed
  desired_state_.position = 0.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // Check defining only max_position soft_limit (shouldn't consider it)
  soft_limits.max_position = 3.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 4.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), limits.max_position, COMMON_THRESHOLD);
  // Check defining max_position soft_limit equal to min_position soft_limit (shouldn't consider it)
  soft_limits.min_position = 3.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 4.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), limits.max_position, COMMON_THRESHOLD);
  // Check defining min_postion soft_limit greater than max_position soft_limit (shouldn't consider
  // it)
  soft_limits.min_position = 5.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 4.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), limits.max_position, COMMON_THRESHOLD);
  // Check defining correct limits (lower than hard_limits). It should clamp to soft limits.
  soft_limits.min_position = -1.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 4.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), soft_limits.max_position, COMMON_THRESHOLD);
  // If desired is inside the limits shouldn't clamp.
  desired_state_.position = 0.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);
  // Clamp to min_position soft_limits
  desired_state_.position = -2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), soft_limits.min_position, COMMON_THRESHOLD);

  // Now add the velocity limits
  soft_limits = joint_limits::SoftJointLimits();
  limits.max_velocity = 1.0;
  limits.has_velocity_limits = true;
  ASSERT_TRUE(Init(limits, soft_limits));
  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};
  desired_state_.position = 0.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);
  EXPECT_TRUE(desired_state_.has_position());
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // Let's set the desired position greater than reachable with max velocity limit
  desired_state_.position = 2.0;
  // As per max velocity limit, it can only reach 1.0 in 1 second
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  // As per max velocity limit, it can only reach 1.0 in 1 second

  // If soft position limits are less restrictive consider velocity_limits
  soft_limits.max_position = 1.5;
  soft_limits.min_position = -1.5;
  ASSERT_TRUE(Init(limits, soft_limits));
  actual_state_.position = 0.0;
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  // If soft position limits are more restrictive consider soft position limits
  soft_limits.max_position = 0.75;
  soft_limits.min_position = -0.75;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 2.0;
  actual_state_ = {};
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), soft_limits.max_position, COMMON_THRESHOLD);
  desired_state_.position = -3.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  // As per max velocity limit, it can only reach -0.25 in 1 second
  EXPECT_NEAR(desired_state_.position.value(), -0.25, COMMON_THRESHOLD);

  // Now let's check the case where the actual position is at 2.0 and the desired position is -M_PI
  // with max velocity limit of 1.0
  soft_limits = joint_limits::SoftJointLimits();
  ASSERT_TRUE(Init(limits, soft_limits));
  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};
  actual_state_.position = 2.0;
  desired_state_.position = -M_PI;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  EXPECT_TRUE(desired_state_.has_position());
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  soft_limits.min_position = 1.5;
  soft_limits.max_position = 3.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_ = {};
  actual_state_ = {};
  actual_state_.position = 2.0;
  desired_state_.position = -M_PI;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), soft_limits.min_position, COMMON_THRESHOLD);

  // Check using k_position
  soft_limits = joint_limits::SoftJointLimits();
  soft_limits.k_position = 1.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 0.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  actual_state_.position = 0.95;
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.95, COMMON_THRESHOLD);
  actual_state_.position = 1.5;
  desired_state_.position = 2.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 2.0, COMMON_THRESHOLD);

  soft_limits.k_position = 1.0;
  soft_limits.max_position = 1.5;
  soft_limits.min_position = -1.5;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 0.0;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  actual_state_.position = 0.45;
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.45, COMMON_THRESHOLD);
  actual_state_.position = 0.95;
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.5, COMMON_THRESHOLD);

  // The case of no actual position feedback
  soft_limits.k_position = 0.5;
  soft_limits.max_position = 1.5;
  soft_limits.min_position = -1.5;
  ASSERT_TRUE(Init(limits, soft_limits));
  actual_state_.position = std::nullopt;
  desired_state_.position = 0.2;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.2, COMMON_THRESHOLD);
  auto prev_command_state = desired_state_;
  while (desired_state_.position.value() < (soft_limits.max_position - COMMON_THRESHOLD))
  {
    desired_state_.position = 2.0;
    double expected_pos =
      prev_command_state.position.value() +
      (soft_limits.max_position - prev_command_state.position.value()) * soft_limits.k_position;
    ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_NEAR(desired_state_.position.value(), expected_pos, COMMON_THRESHOLD);
    prev_command_state = desired_state_;
  }
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.5, COMMON_THRESHOLD);

  // More generic test case
  soft_limits.k_position = 0.5;
  soft_limits.max_position = 2.0;
  soft_limits.min_position = -2.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  actual_state_.position = 0.2;
  while (actual_state_.position.value() < (desired_state_.position.value() - COMMON_THRESHOLD))
  {
    desired_state_.position = 2.0;
    double expected_pos =
      actual_state_.position.value() +
      (soft_limits.max_position - actual_state_.position.value()) * soft_limits.k_position;
    ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_NEAR(desired_state_.position.value(), expected_pos, COMMON_THRESHOLD);
    actual_state_.position = expected_pos;
  }

  // More generic test case to mock a slow system
  soft_limits.k_position = 0.5;
  soft_limits.max_position = 2.0;
  soft_limits.min_position = -2.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  desired_state_.position = 2.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 1.0, COMMON_THRESHOLD);
  actual_state_.position = -0.1;
  for (auto i = 0; i < 10000; i++)
  {
    SCOPED_TRACE(
      "Testing for iteration: " + std::to_string(i) +
      " for desired position: " + std::to_string(desired_state_.position.value()) +
      " and actual position : " + std::to_string(actual_state_.position.value()) +
      " for the joint limits : " + limits.to_string());
    desired_state_.position = 4.0;
    const double delta_pos = std::min(
      (soft_limits.max_position - actual_state_.position.value()) * soft_limits.k_position,
      limits.max_velocity * period.seconds());
    const double expected_pos = actual_state_.position.value() + delta_pos;
    ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_NEAR(desired_state_.position.value(), expected_pos, COMMON_THRESHOLD);
    actual_state_.position = expected_pos / 1.27;
  }

  // Now test when there are no position limits and soft limits, then the desired position is not
  // saturated
  limits = joint_limits::JointLimits();
  soft_limits = joint_limits::SoftJointLimits();
  ASSERT_TRUE(Init(limits, soft_limits));
  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};
  actual_state_.position = 2.0;
  desired_state_.position = 5.0 * M_PI;
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 5.0 * M_PI, COMMON_THRESHOLD);
  EXPECT_TRUE(desired_state_.has_position());
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());
}

TEST_F(JointSoftLimiterTest, check_desired_velocity_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -5.0;
  limits.max_position = 5.0;
  limits.has_velocity_limits = true;
  limits.max_velocity = 1.0;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  // no size check occurs (yet) so expect true
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};

  rclcpp::Duration period(1, 0);  // 1 second
  desired_state_.velocity = 2.0;
  EXPECT_FALSE(desired_state_.has_position());
  EXPECT_TRUE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_FALSE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // Now as the position limits are already configure, set the actual position nearby the limits,
  // then the max velocity needs to adapt wrt to the position limits
  // It is saturated as the position reported is close to the position limits
  auto test_limit_enforcing = [&](
                                const std::optional<double> & actual_position,
                                double desired_velocity, double expected_velocity, bool is_clamped)
  {
    // Reset the desired and actual states
    desired_state_ = {};
    actual_state_ = {};
    const double act_pos = actual_position.has_value() ? actual_position.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    SCOPED_TRACE(
      "Testing for actual position: " + std::to_string(act_pos) +
      ", desired velocity: " + std::to_string(desired_velocity) + ", expected velocity: " +
      std::to_string(expected_velocity) + ", is clamped: " + std::to_string(is_clamped) +
      " for the joint limits : " + limits.to_string());
    if (actual_position.has_value())
    {
      actual_state_.position = actual_position.value();
    }
    desired_state_.velocity = desired_velocity;
    ASSERT_EQ(is_clamped, joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_FALSE(desired_state_.has_position());
    EXPECT_TRUE(desired_state_.has_velocity());
    EXPECT_NEAR(desired_state_.velocity.value(), expected_velocity, COMMON_THRESHOLD);
    EXPECT_FALSE(desired_state_.has_acceleration());
    EXPECT_FALSE(desired_state_.has_effort());
    EXPECT_FALSE(desired_state_.has_jerk());
  };

  auto test_generic_cases = [&](const joint_limits::SoftJointLimits & soft_joint_limits)
  {
    ASSERT_TRUE(Init(limits, soft_joint_limits));

    // Test cases when there is no actual position
    // For hard limits, if there is no actual state but the desired state is outside the limit, then
    // saturate it to the limits
    test_limit_enforcing(std::nullopt, 2.0, 1.0, true);
    test_limit_enforcing(std::nullopt, 1.1, 1.0, true);
    test_limit_enforcing(std::nullopt, -5.0, -1.0, true);
    test_limit_enforcing(std::nullopt, -std::numeric_limits<double>::infinity(), -1.0, true);
    test_limit_enforcing(std::nullopt, std::numeric_limits<double>::infinity(), 1.0, true);
    test_limit_enforcing(std::nullopt, -3.212, -1.0, true);
    test_limit_enforcing(std::nullopt, -0.8, -0.8, false);
    test_limit_enforcing(std::nullopt, 0.8, 0.8, false);
    test_limit_enforcing(std::nullopt, 0.12, 0.12, false);
    test_limit_enforcing(std::nullopt, 0.0, 0.0, false);

    // The cases where the actual position value exist
    test_limit_enforcing(4.5, 5.0, 0.5, true);
    test_limit_enforcing(4.8, 5.0, 0.2, true);
    test_limit_enforcing(4.5, 0.3, 0.3, false);
    test_limit_enforcing(4.5, 0.5, 0.5, false);
    test_limit_enforcing(5.0, 0.9, 0.0, true);
    // When the position is out of the limits, then the velocity is saturated to zero
    test_limit_enforcing(6.0, 2.0, 0.0, true);
    test_limit_enforcing(6.0, -2.0, 0.0, true);
    test_limit_enforcing(4.0, 0.5, 0.5, false);
    test_limit_enforcing(-4.8, -6.0, -0.2, true);
    test_limit_enforcing(4.3, 5.0, 0.7, true);
    test_limit_enforcing(-4.5, -5.0, -0.5, true);
    test_limit_enforcing(-4.5, -0.2, -0.2, false);
    test_limit_enforcing(-3.0, -5.0, -1.0, true);
    test_limit_enforcing(-3.0, -1.0, -1.0, false);
    test_limit_enforcing(-5.0, -3.0, 0.0, true);
    test_limit_enforcing(-5.0, -1.0, 0.0, true);
    // When the position is out of the limits, then the velocity is saturated to zero
    test_limit_enforcing(-6.0, -1.0, 0.0, true);
    test_limit_enforcing(-6.0, -2.0, 0.0, true);
    test_limit_enforcing(-6.0, 1.0, 0.0, true);
  };

  test_generic_cases(soft_limits);
  soft_limits.k_position = 1.0;
  test_generic_cases(soft_limits);
  soft_limits.max_position = 10.0;
  test_generic_cases(soft_limits);
  soft_limits.min_position = -std::numeric_limits<double>::infinity();
  test_generic_cases(soft_limits);
  soft_limits.min_position = -10.0;
  test_generic_cases(soft_limits);

  soft_limits.k_position = 1.0;
  soft_limits.max_position = 4.0;
  soft_limits.min_position = -2.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  test_limit_enforcing(std::nullopt, 2.0, 1.0, true);
  test_limit_enforcing(std::nullopt, 1.1, 1.0, true);
  test_limit_enforcing(std::nullopt, -5.0, -1.0, true);
  test_limit_enforcing(std::nullopt, -std::numeric_limits<double>::infinity(), -1.0, true);
  test_limit_enforcing(std::nullopt, std::numeric_limits<double>::infinity(), 1.0, true);
  test_limit_enforcing(std::nullopt, -3.212, -1.0, true);
  test_limit_enforcing(std::nullopt, -0.8, -0.8, false);
  test_limit_enforcing(std::nullopt, 0.8, 0.8, false);
  test_limit_enforcing(std::nullopt, 0.12, 0.12, false);
  test_limit_enforcing(std::nullopt, 0.0, 0.0, false);

  // The cases where the actual position value exist
  test_limit_enforcing(3.0, 5.0, 1.0, true);
  test_limit_enforcing(3.8, 5.0, 0.2, true);
  test_limit_enforcing(3.5, 0.3, 0.3, false);
  test_limit_enforcing(3.5, 0.5, 0.5, false);
  test_limit_enforcing(3.8, 0.5, 0.2, true);

  test_limit_enforcing(4.0, 0.5, 0.0, true);
  test_limit_enforcing(-2.0, -0.5, 0.0, true);
  test_limit_enforcing(4.0, -0.5, -0.5, false);
  test_limit_enforcing(-2.0, 0.5, 0.5, false);
  test_limit_enforcing(4.0, -3.0, -1.0, true);
  test_limit_enforcing(-2.0, 3.0, 1.0, true);

  test_limit_enforcing(4.8, 0.6, -M_PI / 180., true);
  test_limit_enforcing(4.3, 5.0, -M_PI / 180., true);
  test_limit_enforcing(-4.5, -0.2, M_PI / 180., true);
  test_limit_enforcing(-3.0, -5.0, M_PI / 180., true);

  test_limit_enforcing(4.8, -0.6, -M_PI / 180., true);
  test_limit_enforcing(4.3, -5.0, -M_PI / 180., true);
  test_limit_enforcing(-4.5, 0.2, M_PI / 180., true);
  test_limit_enforcing(-3.0, 5.0, M_PI / 180., true);

  test_limit_enforcing(6.0, 2.0, 0.0, true);
  test_limit_enforcing(6.0, -2.0, 0.0, true);
  test_limit_enforcing(-6.0, -2.0, 0.0, true);
  test_limit_enforcing(-6.0, 2.0, 0.0, true);

  test_limit_enforcing(-5.0, -3.0, M_PI / 180., true);
  test_limit_enforcing(-5.0, -1.0, M_PI / 180., true);
  test_limit_enforcing(5.0, 3.0, -M_PI / 180., true);
  test_limit_enforcing(5.0, 1.0, -M_PI / 180., true);
  test_limit_enforcing(-5.0, 3.0, M_PI / 180., true);
  test_limit_enforcing(-5.0, 1.0, M_PI / 180., true);
  test_limit_enforcing(5.0, -3.0, -M_PI / 180., true);
  test_limit_enforcing(5.0, -1.0, -M_PI / 180., true);

  // Now remove the position limits and only test with acceleration limits
  soft_limits = joint_limits::SoftJointLimits();
  limits.has_position_limits = false;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 0.5;
  // When launching init, the prev_command_ within the limiter will be reset
  ASSERT_TRUE(Init(limits, soft_limits));
  // Now the velocity limits are now saturated by the acceleration limits so in succeeding call it
  // will reach the desired if it is within the max velocity limits. Here, the order of the tests is
  // important.
  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(10.0),
        std::optional<double>(-10.0)})
  {
    test_limit_enforcing(act_pos, 0.0, 0.0, false);  // Helps to reset th prev_command internally
    test_limit_enforcing(act_pos, 1.0, 0.5, true);
    test_limit_enforcing(act_pos, 1.0, 1.0, false);
    test_limit_enforcing(act_pos, -0.2, 0.5, true);
    test_limit_enforcing(act_pos, -0.2, 0.0, true);
    test_limit_enforcing(act_pos, -0.2, -0.2, false);
    test_limit_enforcing(act_pos, -0.3, -0.3, false);
    test_limit_enforcing(act_pos, -0.9, -0.8, true);
    test_limit_enforcing(act_pos, -0.9, -0.9, false);
    test_limit_enforcing(act_pos, -2.0, -1.0, true);
    test_limit_enforcing(act_pos, 2.0, -0.5, true);
    test_limit_enforcing(act_pos, 2.0, 0.0, true);
    test_limit_enforcing(act_pos, 2.0, 0.5, true);
    test_limit_enforcing(act_pos, 2.0, 1.0, true);
    test_limit_enforcing(act_pos, 0.0, 0.5, true);
    test_limit_enforcing(act_pos, 0.0, 0.0, false);
  }

  // Now re-enable the position limits and test with acceleration limits
  limits.has_position_limits = true;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 0.5;
  // When launching init, the prev_command_ within the limiter will be reset
  ASSERT_TRUE(Init(limits, soft_limits));
  // Now the velocity limits are now saturated by the acceleration limits so in succeeding call it
  // will reach the desired if it is within the max velocity limits. Here, the order of the tests is
  // important.
  test_limit_enforcing(4.5, 0.0, 0.0, false);  // Helps to reset th prev_command internally
  test_limit_enforcing(4.5, 1.0, 0.5, true);
  test_limit_enforcing(4.8, 1.0, 0.2, true);
  test_limit_enforcing(4.8, -1.0, -0.3, true);
  test_limit_enforcing(4.8, -1.0, -0.8, true);
  test_limit_enforcing(4.8, -1.0, -1.0, false);
  // here the velocity is saturated by the acceleration limits, if not then the velocity will be
  // directly -0.2
  test_limit_enforcing(-4.8, -2.0, -0.5, true);
  test_limit_enforcing(-4.8, -1.0, -0.2, true);
  test_limit_enforcing(-4.3, -1.0, -0.7, true);
  test_limit_enforcing(-4.3, 0.0, -0.2, true);
  test_limit_enforcing(-4.3, 0.0, 0.0, false);
  test_limit_enforcing(-6.0, 1.0, 0.0, true);
  test_limit_enforcing(-6.0, -1.0, 0.0, true);
  test_limit_enforcing(6.0, 1.0, 0.0, true);
  test_limit_enforcing(6.0, -1.0, 0.0, true);
}

TEST_F(JointSoftLimiterTest, check_desired_effort_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -5.0;
  limits.max_position = 5.0;
  limits.has_velocity_limits = true;
  limits.max_velocity = 1.0;
  limits.has_effort_limits = true;
  limits.max_effort = 200.0;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  // Reset the desired and actual states
  desired_state_ = {};
  actual_state_ = {};

  rclcpp::Duration period(1, 0);  // 1 second
  desired_state_.effort = 20.0;
  EXPECT_FALSE(desired_state_.has_position());
  EXPECT_FALSE(desired_state_.has_velocity());
  EXPECT_FALSE(desired_state_.has_acceleration());
  EXPECT_TRUE(desired_state_.has_effort());
  EXPECT_FALSE(desired_state_.has_jerk());

  // Now as the position limits are already configure, set the actual position nearby the limits,
  // then the max velocity needs to adapt wrt to the position limits
  // It is saturated as the position reported is close to the position limits
  auto test_limit_enforcing = [&](
                                const std::optional<double> & actual_position,
                                const std::optional<double> & actual_velocity,
                                double desired_effort, double expected_effort, bool is_clamped)
  {
    // Reset the desired and actual states
    desired_state_ = {};
    actual_state_ = {};
    const double act_pos = actual_position.has_value() ? actual_position.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    const double act_vel = actual_velocity.has_value() ? actual_velocity.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    SCOPED_TRACE(
      "Testing for actual position: " + std::to_string(act_pos) + ", actual velocity: " +
      std::to_string(act_vel) + ", desired effort: " + std::to_string(desired_effort) +
      ", expected effort: " + std::to_string(expected_effort) + ", is clamped: " +
      std::to_string(is_clamped) + " for the joint limits : " + limits.to_string());
    if (actual_position.has_value())
    {
      actual_state_.position = actual_position.value();
    }
    if (actual_velocity.has_value())
    {
      actual_state_.velocity = actual_velocity.value();
    }
    desired_state_.effort = desired_effort;
    ASSERT_EQ(is_clamped, joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_FALSE(desired_state_.has_position());
    EXPECT_FALSE(desired_state_.has_velocity());
    EXPECT_TRUE(desired_state_.has_effort());
    EXPECT_NEAR(desired_state_.effort.value(), expected_effort, COMMON_THRESHOLD);
    EXPECT_FALSE(desired_state_.has_acceleration());
    EXPECT_FALSE(desired_state_.has_jerk());
  };

  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(4.0),
        std::optional<double>(-4.0)})
  {
    for (auto act_vel :
         {std::optional<double>(std::nullopt), std::optional<double>(0.4),
          std::optional<double>(-0.2)})
    {
      test_limit_enforcing(act_pos, act_vel, 20.0, 20.0, false);
      test_limit_enforcing(act_pos, act_vel, 200.0, 200.0, false);
      test_limit_enforcing(act_pos, act_vel, 201.0, 200.0, true);
      test_limit_enforcing(act_pos, act_vel, 0.0, 0.0, false);
      test_limit_enforcing(act_pos, act_vel, -20.0, -20.0, false);
      test_limit_enforcing(act_pos, act_vel, -200.0, -200.0, false);
      test_limit_enforcing(act_pos, act_vel, -201.0, -200.0, true);
    }
  }

  // The convention is that the positive velocity/position will result in positive effort
  // Now the cases where the actual position or the actual velocity is out of the limits
  test_limit_enforcing(5.0, 0.0, 20.0, 0.0, true);
  test_limit_enforcing(5.0, 0.0, 400.0, 0.0, true);
  test_limit_enforcing(6.0, 0.0, 400.0, 0.0, true);
  test_limit_enforcing(5.0, 0.0, -20.0, -20.0, false);
  test_limit_enforcing(5.0, 0.0, -400.0, -200.0, true);
  test_limit_enforcing(6.0, 0.0, -400.0, -200.0, true);

  // At the limit, when trying to move away from the limit, it should allow
  test_limit_enforcing(5.0, -0.2, 400.0, 200.0, true);
  test_limit_enforcing(5.0, -0.2, -400.0, -200.0, true);
  test_limit_enforcing(5.0, -0.2, 30.0, 30.0, false);
  test_limit_enforcing(5.0, -0.2, -30.0, -30.0, false);
  // For the positive velocity with limit, the effort is saturated
  test_limit_enforcing(5.0, 0.2, 400.0, 0.0, true);
  test_limit_enforcing(5.0, 0.2, 30.0, 0.0, true);
  test_limit_enforcing(5.0, 0.2, -400.0, -200.0, true);
  test_limit_enforcing(5.0, 0.2, -30.0, -30.0, false);

  test_limit_enforcing(-5.0, 0.2, 20.0, 20.0, false);
  test_limit_enforcing(-5.0, 0.2, 400.0, 200.0, true);
  test_limit_enforcing(-5.0, 0.2, -20.0, -20.0, false);
  test_limit_enforcing(-5.0, 0.2, -400.0, -200.0, true);
  // For the negative velocity with limit, the effort is saturated
  test_limit_enforcing(-5.0, -0.2, -400.0, 0.0, true);
  test_limit_enforcing(-5.0, -0.2, -30.0, 0.0, true);
  test_limit_enforcing(-5.0, -0.2, 400.0, 200.0, true);
  test_limit_enforcing(-5.0, -0.2, 30.0, 30.0, false);

  // Check with high values of k_velocity (hard limits should be considered)
  soft_limits.k_velocity = 5000.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(4.0),
        std::optional<double>(-4.0)})
  {
    for (auto act_vel :
         {std::optional<double>(std::nullopt), std::optional<double>(0.5),
          std::optional<double>(-0.2)})
    {
      test_limit_enforcing(act_pos, act_vel, 20.0, 20.0, false);
      test_limit_enforcing(act_pos, act_vel, 200.0, 200.0, false);
      test_limit_enforcing(act_pos, act_vel, 201.0, 200.0, true);
      test_limit_enforcing(act_pos, act_vel, 0.0, 0.0, false);
      test_limit_enforcing(act_pos, act_vel, -20.0, -20.0, false);
      test_limit_enforcing(act_pos, act_vel, -200.0, -200.0, false);
      test_limit_enforcing(act_pos, act_vel, -201.0, -200.0, true);
    }
  }

  // Check with low values of k_velocity
  soft_limits.k_velocity = 300.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(4.0),
        std::optional<double>(-4.0)})
  {
    test_limit_enforcing(act_pos, 0.5, 20.0, 20.0, false);
    test_limit_enforcing(act_pos, 0.5, 200.0, 150.0, true);
    test_limit_enforcing(act_pos, 0.5, 201.0, 150.0, true);
    test_limit_enforcing(act_pos, 0.5, 0.0, 0.0, false);
    test_limit_enforcing(act_pos, 0.5, -20.0, -20.0, false);
    test_limit_enforcing(act_pos, 0.5, -200.0, -200.0, false);
    test_limit_enforcing(act_pos, 0.5, -201.0, -200.0, true);
    test_limit_enforcing(act_pos, -0.5, 20.0, 20.0, false);
    test_limit_enforcing(act_pos, -0.5, 200.0, 200.0, false);
    test_limit_enforcing(act_pos, -0.5, 201.0, 200.0, true);
    test_limit_enforcing(act_pos, -0.5, 0.0, 0.0, false);
    test_limit_enforcing(act_pos, -0.5, -20.0, -20.0, false);
    test_limit_enforcing(act_pos, -0.5, -200.0, -150.0, true);
    test_limit_enforcing(act_pos, -0.5, -201.0, -150.0, true);
  }

  // Check with low values of k_velocity and low soft_limits
  soft_limits.k_velocity = 300.0;
  soft_limits.k_position = 1.0;
  soft_limits.min_position = -4.0;
  soft_limits.max_position = 4.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  test_limit_enforcing(0.0, 0.5, 20.0, 20.0, false);
  test_limit_enforcing(0.0, 0.5, 200.0, 150.0, true);
  test_limit_enforcing(0.0, 0.5, 201.0, 150.0, true);
  test_limit_enforcing(0.0, 0.5, 0.0, 0.0, false);
  test_limit_enforcing(0.0, 0.5, -20.0, -20.0, false);
  test_limit_enforcing(0.0, 0.5, -200.0, -200.0, false);
  test_limit_enforcing(0.0, 0.5, -201.0, -200.0, true);

  test_limit_enforcing(0.0, -0.5, 20.0, 20.0, false);
  test_limit_enforcing(0.0, -0.5, 200.0, 200.0, false);
  test_limit_enforcing(0.0, -0.5, 201.0, 200.0, true);
  test_limit_enforcing(0.0, -0.5, 0.0, 0.0, false);
  test_limit_enforcing(0.0, -0.5, -20.0, -20.0, false);
  test_limit_enforcing(0.0, -0.5, -200.0, -150.0, true);
  test_limit_enforcing(0.0, -0.5, -201.0, -150.0, true);

  // Close to the soft limit with a velocity moving away from the limit
  test_limit_enforcing(-3.5, 0.5, 20.0, 20.0, false);
  test_limit_enforcing(-3.5, 0.5, 200.0, 150.0, true);
  test_limit_enforcing(-3.5, 0.5, 201.0, 150.0, true);
  test_limit_enforcing(-3.5, 0.5, 0.0, 0.0, false);
  test_limit_enforcing(-3.5, 0.5, -20.0, -20.0, false);
  test_limit_enforcing(-3.5, 0.5, -200.0, -200.0, false);
  test_limit_enforcing(-3.5, 0.5, -201.0, -200.0, true);

  test_limit_enforcing(3.5, -0.5, 20.0, 20.0, false);
  test_limit_enforcing(3.5, -0.5, 200.0, 200.0, false);
  test_limit_enforcing(3.5, -0.5, 201.0, 200.0, true);
  test_limit_enforcing(3.5, -0.5, 0.0, 0.0, false);
  test_limit_enforcing(3.5, -0.5, -20.0, -20.0, false);
  test_limit_enforcing(3.5, -0.5, -200.0, -150.0, true);
  test_limit_enforcing(3.5, -0.5, -201.0, -150.0, true);

  // Close to the soft limit with a velocity moving close to the limit
  test_limit_enforcing(3.5, 0.4, 20.0, 20.0, false);
  test_limit_enforcing(3.5, 0.4, 200.0, 30.0, true);
  test_limit_enforcing(3.5, 0.5, 200.0, 0.0, true);
  test_limit_enforcing(3.5, 0.4, 201.0, 30.0, true);
  test_limit_enforcing(3.5, 0.4, 0.0, 0.0, false);
  test_limit_enforcing(3.5, 0.4, -20.0, -20.0, false);
  test_limit_enforcing(3.5, 0.4, -200.0, -200.0, false);
  test_limit_enforcing(3.5, 0.4, -201.0, -200.0, true);

  test_limit_enforcing(-3.5, -0.4, 20.0, 20.0, false);
  test_limit_enforcing(-3.5, -0.4, 200.0, 200.0, false);
  test_limit_enforcing(-3.5, -0.4, 201.0, 200.0, true);
  test_limit_enforcing(-3.5, -0.4, 0.0, 0.0, false);
  test_limit_enforcing(-3.5, -0.4, -20.0, -20.0, false);
  test_limit_enforcing(-3.5, -0.4, -200.0, -30.0, true);
  test_limit_enforcing(-3.5, -0.5, -200.0, 0.0, true);
  test_limit_enforcing(-3.5, -0.4, -201.0, -30.0, true);

  // Check with high values of k_velocity and high soft limits (higher than hard limits)
  soft_limits.k_velocity = 500.0;
  soft_limits.k_position = 1.0;
  soft_limits.min_position = -10.0;
  soft_limits.max_position = 10.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(4.0),
        std::optional<double>(-4.0)})
  {
    for (auto act_vel :
         {std::optional<double>(std::nullopt), std::optional<double>(0.5),
          std::optional<double>(-0.2)})
    {
      test_limit_enforcing(act_pos, act_vel, 20.0, 20.0, false);
      test_limit_enforcing(act_pos, act_vel, 200.0, 200.0, false);
      test_limit_enforcing(act_pos, act_vel, 201.0, 200.0, true);
      test_limit_enforcing(act_pos, act_vel, 0.0, 0.0, false);
      test_limit_enforcing(act_pos, act_vel, -20.0, -20.0, false);
      test_limit_enforcing(act_pos, act_vel, -200.0, -200.0, false);
      test_limit_enforcing(act_pos, act_vel, -201.0, -200.0, true);
    }
  }

  // Check with low values of k_velocity and high soft limits (higher than hard limits)
  soft_limits.k_velocity = 300.0;
  soft_limits.k_position = 1.0;
  soft_limits.min_position = -10.0;
  soft_limits.max_position = 10.0;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  for (auto act_pos :
       {std::optional<double>(std::nullopt), std::optional<double>(4.0),
        std::optional<double>(-4.0)})
  {
    test_limit_enforcing(act_pos, 0.5, 20.0, 20.0, false);
    test_limit_enforcing(act_pos, 0.5, 200.0, 150.0, true);
    test_limit_enforcing(act_pos, 0.5, 201.0, 150.0, true);
    test_limit_enforcing(act_pos, 0.5, 0.0, 0.0, false);
    test_limit_enforcing(act_pos, 0.5, -20.0, -20.0, false);
    test_limit_enforcing(act_pos, 0.5, -200.0, -200.0, false);
    test_limit_enforcing(act_pos, 0.5, -201.0, -200.0, true);

    test_limit_enforcing(act_pos, -0.5, 20.0, 20.0, false);
    test_limit_enforcing(act_pos, -0.5, 200.0, 200.0, false);
    test_limit_enforcing(act_pos, -0.5, 201.0, 200.0, true);
    test_limit_enforcing(act_pos, -0.5, 0.0, 0.0, false);
    test_limit_enforcing(act_pos, -0.5, -20.0, -20.0, false);
    test_limit_enforcing(act_pos, -0.5, -200.0, -150.0, true);
    test_limit_enforcing(act_pos, -0.5, -201.0, -150.0, true);
  }
}

TEST_F(JointSoftLimiterTest, check_desired_acceleration_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 0.5;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  rclcpp::Duration period(1, 0);  // 1 second
  auto test_limit_enforcing = [&](
                                const std::optional<double> & actual_velocity, double desired_accel,
                                double expected_accel, bool is_clamped)
  {
    // Reset the desired and actual states
    desired_state_ = {};
    actual_state_ = {};
    const double act_vel = actual_velocity.has_value() ? actual_velocity.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    SCOPED_TRACE(
      "Testing for actual velocity: " + std::to_string(act_vel) + ", desired acceleration: " +
      std::to_string(desired_accel) + ", expected acceleration: " + std::to_string(expected_accel) +
      ", is clamped: " + std::to_string(is_clamped) +
      " for the joint limits : " + limits.to_string());
    if (actual_velocity.has_value())
    {
      actual_state_.velocity = actual_velocity.value();
    }
    desired_state_.acceleration = desired_accel;
    ASSERT_EQ(is_clamped, joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_FALSE(desired_state_.has_position());
    EXPECT_FALSE(desired_state_.has_velocity());
    EXPECT_FALSE(desired_state_.has_effort());
    EXPECT_TRUE(desired_state_.has_acceleration());
    EXPECT_NEAR(desired_state_.acceleration.value(), expected_accel, COMMON_THRESHOLD);
    EXPECT_FALSE(desired_state_.has_jerk());
  };

  // Tests without applying deceleration limits
  for (auto act_vel :
       {std::optional<double>(std::nullopt), std::optional<double>(0.4),
        std::optional<double>(-0.2)})
  {
    test_limit_enforcing(act_vel, 0.0, 0.0, false);
    test_limit_enforcing(act_vel, 0.5, 0.5, false);
    test_limit_enforcing(act_vel, 0.6, 0.5, true);
    test_limit_enforcing(act_vel, 1.5, 0.5, true);
    test_limit_enforcing(act_vel, -0.5, -0.5, false);
    test_limit_enforcing(act_vel, -0.6, -0.5, true);
    test_limit_enforcing(act_vel, -1.5, -0.5, true);
  }

  // Now let's test with applying deceleration limits
  limits.has_deceleration_limits = true;
  limits.max_deceleration = 0.25;
  // When launching init, the prev_command_ within the limiter will be reset
  ASSERT_TRUE(Init(limits, soft_limits));

  // If you don't have the actual velocity, the deceleration limits are not applied
  test_limit_enforcing(std::nullopt, 0.0, 0.0, false);
  test_limit_enforcing(std::nullopt, 0.5, 0.5, false);
  test_limit_enforcing(std::nullopt, 0.6, 0.5, true);
  test_limit_enforcing(std::nullopt, 1.5, 0.5, true);
  test_limit_enforcing(std::nullopt, -0.5, -0.5, false);
  test_limit_enforcing(std::nullopt, -0.6, -0.5, true);
  test_limit_enforcing(std::nullopt, -1.5, -0.5, true);
  test_limit_enforcing(std::nullopt, 0.0, 0.0, false);

  // Testing both positive and negative velocities and accelerations together without a proper
  // deceleration
  test_limit_enforcing(0.4, 0.2, 0.2, false);
  test_limit_enforcing(0.4, 0.8, 0.5, true);
  test_limit_enforcing(-0.4, -0.2, -0.2, false);
  test_limit_enforcing(-0.4, -0.6, -0.5, true);

  // The deceleration limits are basically applied when the acceleration is positive and the
  // velocity is negative and when the acceleration is negative and the velocity is positive
  test_limit_enforcing(0.4, -0.1, -0.1, false);
  test_limit_enforcing(0.4, -0.25, -0.25, false);
  test_limit_enforcing(0.4, -0.6, -0.25, true);
  test_limit_enforcing(0.4, -4.0, -0.25, true);
  test_limit_enforcing(-0.4, 0.1, 0.1, false);
  test_limit_enforcing(-0.4, 0.25, 0.25, false);
  test_limit_enforcing(-0.4, 0.6, 0.25, true);
  test_limit_enforcing(-0.4, 3.0, 0.25, true);
}

TEST_F(JointSoftLimiterTest, check_desired_jerk_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_jerk_limits = true;
  limits.max_jerk = 0.5;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  rclcpp::Duration period(1, 0);  // 1 second
  auto test_limit_enforcing = [&](double desired_jerk, double expected_jerk, bool is_clamped)
  {
    // Reset the desired and actual states
    desired_state_ = {};
    actual_state_ = {};
    SCOPED_TRACE(
      "Testing for desired jerk : " + std::to_string(desired_jerk) + ", expected jerk: " +
      std::to_string(expected_jerk) + ", is clamped: " + std::to_string(is_clamped) +
      " for the joint limits : " + limits.to_string());
    desired_state_.jerk = desired_jerk;
    ASSERT_EQ(is_clamped, joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_FALSE(desired_state_.has_position());
    EXPECT_FALSE(desired_state_.has_velocity());
    EXPECT_FALSE(desired_state_.has_effort());
    EXPECT_FALSE(desired_state_.has_acceleration());
    EXPECT_TRUE(desired_state_.has_jerk());
    EXPECT_NEAR(desired_state_.jerk.value(), expected_jerk, COMMON_THRESHOLD);
  };

  // Check with jerk limits
  test_limit_enforcing(0.0, 0.0, false);
  test_limit_enforcing(0.5, 0.5, false);
  test_limit_enforcing(0.6, 0.5, true);
  test_limit_enforcing(1.5, 0.5, true);
  test_limit_enforcing(-0.5, -0.5, false);
  test_limit_enforcing(-0.6, -0.5, true);
  test_limit_enforcing(-1.5, -0.5, true);
}

TEST_F(JointSoftLimiterTest, check_all_desired_references_limiting)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -5.0;
  limits.max_position = 5.0;
  limits.has_velocity_limits = true;
  limits.max_velocity = 1.0;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 0.5;
  limits.has_deceleration_limits = true;
  limits.max_deceleration = 0.25;
  limits.has_jerk_limits = true;
  limits.max_jerk = 2.0;
  joint_limits::SoftJointLimits soft_limits;
  ASSERT_TRUE(Init(limits, soft_limits));
  ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

  rclcpp::Duration period(1, 0);  // 1 second
  auto test_limit_enforcing =
    [&](
      const std::optional<double> & actual_position, const std::optional<double> & actual_velocity,
      double desired_position, double desired_velocity, double desired_acceleration,
      double desired_jerk, double expected_position, double expected_velocity,
      double expected_acceleration, double expected_jerk, bool is_clamped)
  {
    // Reset the desired and actual states
    desired_state_ = {};
    actual_state_ = {};
    const double act_pos = actual_position.has_value() ? actual_position.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    const double act_vel = actual_velocity.has_value() ? actual_velocity.value()
                                                       : std::numeric_limits<double>::quiet_NaN();
    SCOPED_TRACE(
      "Testing for actual position: " + std::to_string(act_pos) + ", actual velocity: " +
      std::to_string(act_vel) + ", desired position: " + std::to_string(desired_position) +
      ", desired velocity: " + std::to_string(desired_velocity) + ", desired acceleration: " +
      std::to_string(desired_acceleration) + ", desired jerk: " + std::to_string(desired_jerk) +
      ", expected position: " + std::to_string(expected_position) +
      ", expected velocity: " + std::to_string(expected_velocity) + ", expected acceleration: " +
      std::to_string(expected_acceleration) + ", expected jerk: " + std::to_string(expected_jerk) +
      ", is clamped: " + std::to_string(is_clamped) +
      " for the joint limits : " + limits.to_string());
    if (actual_position.has_value())
    {
      actual_state_.position = actual_position.value();
    }
    if (actual_velocity.has_value())
    {
      actual_state_.velocity = actual_velocity.value();
    }
    desired_state_.position = desired_position;
    desired_state_.velocity = desired_velocity;
    desired_state_.acceleration = desired_acceleration;
    desired_state_.jerk = desired_jerk;
    ASSERT_EQ(is_clamped, joint_limiter_->enforce(actual_state_, desired_state_, period));
    EXPECT_NEAR(desired_state_.position.value(), expected_position, COMMON_THRESHOLD);
    EXPECT_NEAR(desired_state_.velocity.value(), expected_velocity, COMMON_THRESHOLD);
    EXPECT_NEAR(desired_state_.acceleration.value(), expected_acceleration, COMMON_THRESHOLD);
    EXPECT_NEAR(desired_state_.jerk.value(), expected_jerk, COMMON_THRESHOLD);
  };

  // Test cases when there is no actual position and velocity
  // Desired position and velocity affected due to the acceleration limits
  test_limit_enforcing(std::nullopt, std::nullopt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
  test_limit_enforcing(std::nullopt, std::nullopt, 6.0, 2.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 6.0, 2.0, 1.0, 0.5, 1.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 6.0, 2.0, 1.0, 0.5, 1.5, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 6.0, 2.0, 1.0, 0.5, 2.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 3.0, 2.0, 1.0, 0.5, 2.5, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 3.0, 2.0, 1.0, 0.5, 3.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(std::nullopt, std::nullopt, 3.0, 1.0, 0.5, 0.5, 3.0, 1.0, 0.5, 0.5, false);

  // Now enforce the limits with actual position and velocity
  ASSERT_TRUE(Init(limits, soft_limits));
  // Desired position and velocity affected due to the acceleration limits
  test_limit_enforcing(0.5, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(1.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, true);
  // If the actual position doesn't change, the desired position should not change
  test_limit_enforcing(1.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(1.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(1.2, 0.0, 6.0, 0.0, 0.0, 0.0, 1.7, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(1.5, 0.0, 6.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(1.5, 0.0, -6.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, true);
  test_limit_enforcing(2.0, 0.0, 6.0, 2.0, 1.0, 0.5, 2.5, 0.5, 0.5, 0.5, true);
  test_limit_enforcing(2.0, 0.5, 6.0, 2.0, 1.0, 0.5, 3.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(3.0, 0.5, 6.0, 2.0, 1.0, 0.5, 4.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(4.0, 0.5, 6.0, 2.0, 1.0, 0.5, 5.0, 1.0, 0.5, 0.5, true);
  test_limit_enforcing(4.8, 0.5, 6.0, 2.0, 1.0, 0.5, 5.0, 0.5, 0.5, 0.5, true);
  test_limit_enforcing(5.0, 0.5, 6.0, 2.0, 1.0, 0.5, 5.0, 0.0, 0.5, 0.5, true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
