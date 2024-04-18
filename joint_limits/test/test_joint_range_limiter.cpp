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

/// \author Sai Kishor Kothakota

#include "test_joint_range_limiter.hpp"
#include <limits>

TEST_F(JointSaturationLimiterTest, when_loading_limiter_plugin_expect_loaded)
{
  // Test JointSaturationLimiter loading
  ASSERT_NO_THROW(
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_)));
  ASSERT_NE(joint_limiter_, nullptr);
}

// NOTE: We accept also if there is no limit defined for a joint.
TEST_F(JointSaturationLimiterTest, when_joint_not_found_expect_init_fail)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());
  // initialize the limiter
  std::vector<std::string> joint_names = {"bar_joint"};
  ASSERT_TRUE(joint_limiter_->init(joint_names, node_));
}

TEST_F(JointSaturationLimiterTest, when_invalid_dt_expect_enforce_fail)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  ASSERT_TRUE(Init());
  ASSERT_TRUE(Configure());
  rclcpp::Duration period(0, 0);  // 0 second
  ASSERT_FALSE(joint_limiter_->enforce(actual_state_, desired_state_, period));
}

TEST_F(JointSaturationLimiterTest, check_desired_position_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -M_PI;
  limits.max_position = M_PI;
  ASSERT_TRUE(Init(limits));
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

  // Now add the velocity limits
  limits.max_velocity = 1.0;
  limits.has_velocity_limits = true;
  ASSERT_TRUE(Init(limits));
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

  // As per max velocity limit, it can only reach 0.0 in 1 second
  desired_state_.position = -3.0;
  ASSERT_TRUE(joint_limiter_->enforce(actual_state_, desired_state_, period));
  EXPECT_NEAR(desired_state_.position.value(), 0.0, COMMON_THRESHOLD);

  // Now let's check the case where the actual position is at 2.0 and the desired position is -M_PI
  // with max velocity limit of 1.0
  ASSERT_TRUE(Init(limits));
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

  // Now test when there are no position limits, then the desired position is not saturated
  limits = joint_limits::JointLimits();
  ASSERT_TRUE(Init(limits));
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

TEST_F(JointSaturationLimiterTest, check_desired_velocity_only_cases)
{
  SetupNode("joint_saturation_limiter");
  ASSERT_TRUE(Load());

  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -5.0;
  limits.max_position = 5.0;
  limits.has_velocity_limits = true;
  limits.max_velocity = 1.0;
  ASSERT_TRUE(Init(limits));
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

  // Now remove the position limits and only test with acceleration limits
  limits.has_position_limits = false;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 0.5;
  // When launching init, the prev_command_ within the limiter will be reset
  ASSERT_TRUE(Init(limits));
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
  ASSERT_TRUE(Init(limits));
  // Now the velocity limits are now saturated by the acceleration limits so in succeeding call it
  // will reach the desired if it is within the max velocity limits. Here, the order of the tests is
  // important.
  test_limit_enforcing(4.5, 0.0, 0.0, false);  // Helps to reset th prev_command internally
  test_limit_enforcing(4.5, 1.0, 0.5, true);
  test_limit_enforcing(4.8, 1.0, 0.2, true);
  test_limit_enforcing(4.8, -1.0, -0.3, true);
  test_limit_enforcing(4.8, -1.0, -0.8, true);
  test_limit_enforcing(4.8, -1.0, -1.0, false);
  test_limit_enforcing(-4.8, -1.0, -0.2, true);
  test_limit_enforcing(-4.3, -1.0, -0.7, true);
  test_limit_enforcing(-4.3, 0.0, -0.2, true);
  test_limit_enforcing(-4.3, 0.0, 0.0, false);
  test_limit_enforcing(-6.0, 1.0, 0.0, true);
  test_limit_enforcing(-6.0, -1.0, 0.0, true);
  test_limit_enforcing(6.0, 1.0, 0.0, true);
  test_limit_enforcing(6.0, -1.0, 0.0, true);
}

// TEST_F(JointSaturationLimiterTest, when_no_posstate_expect_enforce_false)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     joint_limiter_->configure(last_commanded_state_);

//     rclcpp::Duration period(1, 0);  // 1 second
//     // test no position interface
//     current_joint_states_.positions.clear();
//     ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // also fail if out of limits
//     desired_joint_states_.positions[0] = 20.0;
//     ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_no_velstate_expect_limiting)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     joint_limiter_->configure(last_commanded_state_);

//     rclcpp::Duration period(1, 0);  // 1 second
//     // test no vel interface
//     current_joint_states_.velocities.clear();
//     ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
//     // also fail if out of limits
//     desired_joint_states_.positions[0] = 20.0;
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_within_limits_expect_no_limits_applied)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second
//     // pos, vel, acc, dec = 1.0, 2.0, 5.0, 7.5

//     // within limits
//     desired_joint_states_.positions[0] = 1.0;
//     desired_joint_states_.velocities[0] = 1.0;  // valid pos derivatite as well
//     ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if no limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       1.0,  // pos unchanged
//       1.0,  // vel unchanged
//       1.0   // acc = vel / 1.0
//     );
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_within_limits_expect_no_limits_applied_with_acc)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second
//     // pos, vel, acc, dec = 1.0, 2.0, 5.0, 7.5

//     // within limits
//     desired_joint_states_.positions[0] = 1.0;
//     desired_joint_states_.velocities[0] = 1.5;     // valid pos derivative as well
//     desired_joint_states_.accelerations[0] = 2.9;  // valid pos derivative as well
//     ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if no limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       1.0,  // pos unchanged
//       1.5,  // vel unchanged
//       2.9   // acc = vel / 1.0
//     );
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_posvel_leads_to_vel_exceeded_expect_limits_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second

//     // pos/vel cmd ifs
//     current_joint_states_.positions[0] = -2.0;
//     desired_joint_states_.positions[0] = 1.0;
//     // desired velocity exceeds although correctly computed from pos derivative
//     desired_joint_states_.velocities[0] = 3.0;
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       0.0,       // pos = pos + max_vel * dt
//       2.0,       // vel limited to max_vel
//       2.0 / 1.0  // acc set to vel change/DT
//     );

//     // check opposite velocity direction (sign copy)
//     current_joint_states_.positions[0] = 2.0;
//     desired_joint_states_.positions[0] = -1.0;
//     // desired velocity exceeds although correctly computed from pos derivative
//     desired_joint_states_.velocities[0] = -3.0;
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if vel and acc limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       0.0,        // pos = pos - max_vel * dt
//       -2.0,       // vel limited to -max_vel
//       -2.0 / 1.0  // acc set to vel change/DT
//     );
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_posonly_leads_to_vel_exceeded_expect_pos_acc_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second

//     // set current velocity close to limits to not be blocked by max acc to reach max
//     current_joint_states_.velocities[0] = 1.9;
//     // desired pos leads to vel exceeded (4.0 / sec instead of 2.0)
//     desired_joint_states_.positions[0] = 4.0;
//     // no vel cmd (will lead to internal computation of velocity)
//     desired_joint_states_.velocities.clear();
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if pos and acc limits applied
//     ASSERT_EQ(desired_joint_states_.positions[0], 2.0);  // pos limited to max_vel * DT
//     // no vel cmd ifs
//     ASSERT_EQ(
//       desired_joint_states_.accelerations[0], (2.0 - 1.9) / 1.0);  // acc set to vel change/DT

//     // check opposite position and direction
//     current_joint_states_.positions[0] = 0.0;
//     current_joint_states_.velocities[0] = -1.9;
//     desired_joint_states_.positions[0] = -4.0;
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if pos and acc limits applied
//     ASSERT_EQ(desired_joint_states_.positions[0], -2.0);  // pos limited to -max_vel * DT
//     // no vel cmd ifs
//     ASSERT_EQ(
//       desired_joint_states_.accelerations[0], (-2.0 + 1.9) / 1.0);  // acc set to vel change/DT
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_velonly_leads_to_vel_exceeded_expect_vel_acc_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second

//     // vel cmd ifs
//     current_joint_states_.positions[0] = -2.0;
//     // set current velocity close to limits to not be blocked by max acc to reach max
//     current_joint_states_.velocities[0] = 1.9;
//     // no pos cmd
//     desired_joint_states_.positions.clear();
//     // desired velocity exceeds
//     desired_joint_states_.velocities[0] = 3.0;

//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if vel and acc limits applied
//     ASSERT_EQ(desired_joint_states_.velocities[0], 2.0);  // vel limited to max_vel
//     // no vel cmd ifs
//     ASSERT_EQ(
//       desired_joint_states_.accelerations[0], (2.0 - 1.9) / 1.0);  // acc set to vel change/DT

//     // check opposite velocity direction
//     current_joint_states_.positions[0] = 2.0;
//     // set current velocity close to limits to not be blocked by max acc to reach max
//     current_joint_states_.velocities[0] = -1.9;
//     // desired velocity exceeds
//     desired_joint_states_.velocities[0] = -3.0;

//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if vel and acc limits applied
//     ASSERT_EQ(desired_joint_states_.velocities[0], -2.0);  // vel limited to -max_vel
//     // no vel cmd ifs
//     ASSERT_EQ(
//       desired_joint_states_.accelerations[0], (-2.0 + 1.9) / 1.0);  // acc set to vel change/DT
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_posonly_exceeded_expect_pos_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(1.0, 0.0);  // 1 second

//     // desired pos exceeds
//     current_joint_states_.positions[0] = 5.0;
//     desired_joint_states_.positions[0] = 20.0;
//     // no velocity interface
//     desired_joint_states_.velocities.clear();
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if pos limits applied
//     ASSERT_EQ(
//       desired_joint_states_.positions[0], 5.0);  // pos limited clamped (was already at limit)
//     // no vel cmd ifs
//     ASSERT_EQ(desired_joint_states_.accelerations[0], 0.0);  // acc set to vel change/DT
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_position_close_to_pos_limit_expect_deceleration_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     // using 0.05 because 1.0 sec invalidates the "small dt integration"
//     rclcpp::Duration period(0, 50000000);  // 0.05 second

//     // close to pos limit should reduce velocity and stop
//     current_joint_states_.positions[0] = 4.851;
//     current_joint_states_.velocities[0] = 1.5;
//     desired_joint_states_.positions[0] = 4.926;
//     desired_joint_states_.velocities[0] = 1.5;

//     // this setup requires 0.15 distance to stop, and 0.2 seconds (so 4 cycles at 0.05)
//     std::vector expected_ret = {true, true, true, false};
//     for (auto i = 0u; i < 4; ++i)
//     {
//       auto previous_vel_request = desired_joint_states_.velocities[0];
//       // expect limits applied until the end stop
//       ASSERT_EQ(
//         joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period),
//         expected_ret[i]);

//       ASSERT_LE(
//         desired_joint_states_.velocities[0],
//         previous_vel_request);  // vel adapted to reach end-stop should be decreasing
//       // NOTE: after the first cycle, vel is reduced and does not trigger stopping position limit
//       // hence no max deceleration anymore...
//       ASSERT_LT(
//         desired_joint_states_.positions[0],
//         5.0 + COMMON_THRESHOLD);  // should decelerate at each cycle and stay below limits
//       ASSERT_LE(desired_joint_states_.accelerations[0], 0.0);  // should decelerate

//       Integrate(period.seconds());

//       ASSERT_LT(current_joint_states_.positions[0], 5.0);  // below joint limit
//     }
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_posvel_leads_to_acc_exceeded_expect_limits_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(0, 50000000);

//     // desired acceleration exceeds
//     current_joint_states_.positions[0] = 0.1;
//     current_joint_states_.velocities[0] = 0.1;
//     desired_joint_states_.positions[0] = 0.125;  // valid pos cmd for the desired velocity
//     desired_joint_states_.velocities[0] = 0.5;   // leads to acc > max acc
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       0.11125,  // pos = double integration from max acc with current state
//       0.35,     // vel limited to vel + max acc * dt
//       5.0       // acc max acc
//     );
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_posonly_leads_to_acc_exceeded_expect_limits_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(0, 50000000);

//     // desired acceleration exceeds
//     current_joint_states_.positions[0] = 0.1;
//     current_joint_states_.velocities[0] = 0.1;
//     desired_joint_states_.positions[0] =
//       0.125;  // pos cmd leads to vel 0.5 that leads to acc > max acc
//     desired_joint_states_.velocities.clear();
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if pos and acc limits applied
//     ASSERT_NEAR(
//       desired_joint_states_.positions[0], 0.111250,
//       COMMON_THRESHOLD);  // pos = double integration from max acc with current state
//     // no vel cmd ifs
//     ASSERT_EQ(desired_joint_states_.accelerations[0], 5.0);  // acc max acc
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_velonly_leads_to_acc_exceeded_expect_limits_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(0, 50000000);

//     // desired acceleration exceeds
//     current_joint_states_.positions[0] = 0.1;
//     current_joint_states_.velocities[0] = 0.1;
//     desired_joint_states_.positions.clear();    //  = 0.125;
//     desired_joint_states_.velocities[0] = 0.5;  // leads to acc > max acc
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if pos and acc limits applied
//     // no pos cmd ifs
//     ASSERT_EQ(desired_joint_states_.velocities[0], 0.35);    // vel limited to vel + max acc * dt
//     ASSERT_EQ(desired_joint_states_.accelerations[0], 5.0);  // acc max acc
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_deceleration_exceeded_expect_dec_enforced)
// {
//   SetupNode("joint_saturation_limiter");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(0, 50000000);

//     // desired deceleration exceeds
//     current_joint_states_.positions[0] = 0.3;
//     current_joint_states_.velocities[0] = 0.5;
//     desired_joint_states_.positions[0] = 0.305;  // valid pos cmd for the desired velocity
//     desired_joint_states_.velocities[0] = 0.1;   // leads to acc < -max dec

//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if vel and acc limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       0.315625,  // pos = double integration from max dec with current state
//       0.125,     // vel limited by vel - max dec * dt
//       -7.5       // acc limited by -max dec
//     );
//   }
// }

// TEST_F(JointSaturationLimiterTest, when_deceleration_exceeded_with_no_maxdec_expect_acc_enforced)
// {
//   SetupNode("joint_saturation_limiter_nodeclimit");
//   Load();

//   if (joint_limiter_)
//   {
//     Init();
//     Configure();

//     rclcpp::Duration period(0, 50000000);

//     // desired deceleration exceeds
//     current_joint_states_.positions[0] = 1.0;
//     current_joint_states_.velocities[0] = 1.0;
//     desired_joint_states_.positions[0] = 1.025;  // valid pos cmd for the desired decreased
//     velocity desired_joint_states_.velocities[0] = 0.5;   // leads to acc > -max acc
//     ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

//     // check if vel and acc limits applied
//     CHECK_STATE_SINGLE_JOINT(
//       desired_joint_states_, 0,
//       1.04375,  // pos = double integration from max acc with current state
//       0.75,     // vel limited by vel-max acc * dt
//       -5.0      // acc limited to -max acc
//     );
//   }
// }

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
