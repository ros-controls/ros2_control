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

/// \author Dr. Denis Stogl, Guillaume Walck

#include "test_ruckig_joint_limiter.hpp"

TEST_F(RuckigJointLimiterTest, when_loading_limiter_plugin_expect_loaded)
{
  // Test RuckigJointLimiterTest loading
  ASSERT_NO_THROW(
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_)));
  ASSERT_NE(joint_limiter_, nullptr);
}

TEST_F(RuckigJointLimiterTest, when_wrong_input_expect_enforce_fail)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    // no size check occurs (yet) so expect true
    ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

    rclcpp::Duration period(1, 0);  // 1 second
    desired_joint_states_.positions.clear();
    // trigger size check with one wrong size
    ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
  }
}

TEST_F(RuckigJointLimiterTest, when_within_limits_expect_no_limits_applied)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib
    // pos, vel, acc, dec = 1.0, 2.0, 5.0, 7.5

    // within limits
    desired_joint_states_.positions[0] = 1.0;
    desired_joint_states_.velocities[0] = 0.01;
    desired_joint_states_.accelerations[0] = 2.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if max jerk applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],     // pos unchanged
      desired_joint_states_.velocities[0],    // vel unchanged
      desired_joint_states_.accelerations[0]  // acc unchanged
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_positive_velocity_exceeded_expect_acc_vel_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired pos exceeds
    desired_joint_states_.positions[0] = 1.0;
    desired_joint_states_.velocities[0] = 3.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if pos limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                    // vel limited to max_acc * dt / 2.0
      100.0 * period.seconds()  // acc set to max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_negative_velocity_exceeded_expect_acc_vel_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired pos exceeds
    desired_joint_states_.positions[0] = -1.0;
    desired_joint_states_.velocities[0] = -3.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if pos limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                     // vel limited to max_acc * dt / 2.0
      -100.0 * period.seconds()  // acc set to max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_incr_vel_leads_to_acc_exceeded_expect_vel_and_acc_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired vel makes acceleration exceed
    desired_joint_states_.positions[0] = 1.0;
    desired_joint_states_.velocities[0] = 1.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                    // vel limited to max_acc * dt / 2.0
      100.0 * period.seconds()  // acc set to max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_decr_vel_leads_to_acc_exceeded_expect_vel_and_acc_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired vel makes acceleration exceed
    desired_joint_states_.positions[0] = -1.0;
    desired_joint_states_.velocities[0] = -1.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                     // vel limited to max_acc * dt / 2.0
      -100.0 * period.seconds()  // acc set to max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_position_exceeded_expect_nochange)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(1.0, 0.0);  // 1 second

    // desired pos exceeds
    desired_joint_states_.positions[0] = 20.0;
    desired_joint_states_.velocities[0] = 0.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if pos limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],     // pos unchanged
      desired_joint_states_.velocities[0],    // vel unchanged
      desired_joint_states_.accelerations[0]  // acc unchanged
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_acc_within_jerk_limit_exceeded_expect_vel_and_acc_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired acc exceed but within jerk limits
    current_joint_states_.velocities[0] = 1.0;
    current_joint_states_.accelerations[0] = 4.6;
    // required to initialize the limiter correctly
    joint_limiter_->configure(current_joint_states_);
    desired_joint_states_.positions[0] = 1.0;
    desired_joint_states_.velocities[0] = 1.023;
    desired_joint_states_.accelerations[0] = 5.1;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // acc should be close to 5.0
    ASSERT_GT(desired_joint_states_.accelerations[0], 4.6);
    ASSERT_LE(desired_joint_states_.accelerations[0], 5.0);
  }
}

TEST_F(RuckigJointLimiterTest, when_dec_within_jerk_limit_exceeded_expect_vel_and_acc_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second AS HARDCODED in the lib

    // desired acc exceed but within jerk limits
    current_joint_states_.velocities[0] = -1.0;
    current_joint_states_.accelerations[0] = -4.6;
    // required to initialize the limiter correctly
    joint_limiter_->configure(current_joint_states_);
    desired_joint_states_.positions[0] = -1.0;
    desired_joint_states_.velocities[0] = -1.023;
    desired_joint_states_.accelerations[0] = -5.1;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // acc should be close to -5.0
    ASSERT_LT(desired_joint_states_.accelerations[0], -4.6);
    ASSERT_GE(desired_joint_states_.accelerations[0], -5.0);
  }
}

TEST_F(RuckigJointLimiterTest, when_acc_leads_to_jerk_exceeded_expect_acc_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second

    // desired acceleration exceeds
    desired_joint_states_.velocities[0] = 0.002;   // if no velocity set, acc is computed negative !
    desired_joint_states_.accelerations[0] = 0.6;  // leads to jerk > max_jerk
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                    // vel limited to max_acc * dt / 2.0
      100.0 * period.seconds()  // acc is max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_dec_leads_to_jerk_exceeded_expect_dec_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 5000000);  // 0.005 second

    // desired deceleration exceeds
    desired_joint_states_.velocities[0] = -0.002;  // if no velocity set, acc is computed positive !
    desired_joint_states_.accelerations[0] = -0.6;  // leads to jerk > max_jerk
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      // TODO(gwalck) why is the result divided by 2.0 ?
      desired_joint_states_.accelerations[0] * period.seconds() /
        2.0,                     // vel limited to max_acc * dt / 2.0
      -100.0 * period.seconds()  // acc is max_jerk * dt
    );
  }
}

TEST_F(RuckigJointLimiterTest, when_vel_close_to_vel_limit_expect_deceleration_enforced)
{
  SetupNode("ruckig_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    // using 0.05 because 1.0 sec invalidates the "small dt integration"
    rclcpp::Duration period(0, 5000000);  // 0.005 second

    // close to vel limit should trigger braking
    current_joint_states_.positions[0] = 1.0;
    current_joint_states_.velocities[0] = 1.9;
    // required to initialize the limiter correctly
    joint_limiter_->configure(current_joint_states_);
    desired_joint_states_.positions[0] = 1.5;
    desired_joint_states_.velocities[0] = 1.98;
    desired_joint_states_.accelerations[0] = 5.0;

    for (auto i = 0u; i < 4; ++i)
    {
      auto previous_vel_request = desired_joint_states_.velocities[0];
      ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

      ASSERT_LE(
        desired_joint_states_.velocities[0],
        previous_vel_request);  // vel adapted to stay below or equal to previous vel

      ASSERT_LE(desired_joint_states_.velocities[0],
                2.0);  // vel adapted to stay below max vel

      ASSERT_LE(desired_joint_states_.accelerations[0], 0.0);  // should decelerate

      Integrate(period.seconds());

      ASSERT_LT(current_joint_states_.positions[0], 5.0);  // below joint limit
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
