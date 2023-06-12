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

#include "test_simple_joint_limiter.hpp"

TEST_F(SimpleJointLimiterTest, when_loading_limiter_plugin_expect_loaded)
{
  // Test SimpleJointLimiter loading
  ASSERT_NO_THROW(
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_)));
  ASSERT_NE(joint_limiter_, nullptr);
}

/* FIXME: currently init does not check if limit parameters exist for the requested joint
TEST_F(SimpleJointLimiterTest, when_joint_not_found_expect_init_fail)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    // initialize the limiter
    std::vector<std::string> joint_names = {"bar_joint"};
    ASSERT_FALSE(joint_limiter_->init(joint_names, node_));
  }
}
*/

TEST_F(SimpleJointLimiterTest, when_invalid_dt_expect_enforce_fail)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();
    rclcpp::Duration period(0, 0);  // 0 second
    ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
  }
}

TEST_F(SimpleJointLimiterTest, when_wrong_input_expect_enforce_fail)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    // no size check occurs (yet) so expect true
    ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));

    rclcpp::Duration period(1, 0);  // 1 second
    desired_joint_states_.velocities.clear();
    // trigger size check with one wrong size
    ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
  }
}

TEST_F(SimpleJointLimiterTest, when_within_limits_expect_no_limits_applied)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(1.0, 0.0);  // 1 second
    // pos, vel, acc, dec = 1.0, 2.0, 5.0, 7.5

    // within limits
    desired_joint_states_.positions[0] = 1.0;
    desired_joint_states_.velocities[0] = 1.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if no limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],   // pos unchanged
      desired_joint_states_.velocities[0],  // vel unchanged
      desired_joint_states_.velocities[0]   // acc = vel / 1.0
    );
  }
}

TEST_F(SimpleJointLimiterTest, when_velocity_exceeded_expect_vel_and_acc_enforced)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(1.0, 0.0);  // 1 second

    // desired velocity exceeds
    desired_joint_states_.velocities[0] = 3.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      2.0,                                 // vel limited to max_vel
      2.0 / 1.0                            // acc set to vel change/DT
    );

    // check opposite velocity direction (sign copy)
    desired_joint_states_.velocities[0] = -3.0;
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      -2.0,                                // vel limited to -max_vel
      -2.0 / 1.0                           // acc set to vel change/DT
    );
  }
}

TEST_F(SimpleJointLimiterTest, when_position_exceeded_expect_pos_enforced)
{
  SetupNode("simple_joint_limiter");
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
      5.0,                                    // pos unchanged
      desired_joint_states_.velocities[0],    // vel unchanged
      desired_joint_states_.accelerations[0]  // acc unchanged
    );
  }
}

TEST_F(SimpleJointLimiterTest, when_position_close_to_pos_limit_expect_deceleration_enforced)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    // using 0.05 because 1.0 sec invalidates the "small dt integration"
    rclcpp::Duration period(0, 50000000);  // 0.05 second

    // close to pos limit should reduce velocity and stop
    current_joint_states_.positions[0] = 4.851;
    current_joint_states_.velocities[0] = 1.5;
    desired_joint_states_.positions[0] = 4.926;
    desired_joint_states_.velocities[0] = 1.5;

    // this setup requires 0.15 distance to stop, and 0.2 seconds (so 4 cycles at 0.05)
    for (auto i = 0u; i < 4; ++i)
    {
      auto previous_vel_request = desired_joint_states_.velocities[0];
      ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

      ASSERT_LE(
        desired_joint_states_.velocities[0],
        previous_vel_request);  // vel adapted to reach end-stop should be decreasing
      // NOTE: after the first cycle, vel is reduced and does not trigger stopping position limit
      // hence no max deceleration anymore...
      ASSERT_LE(
        desired_joint_states_.positions[0],
        5.0 +
          10.0 *
            COMMON_THRESHOLD);  // is not decelerating at max all the time, hence err in last cycle
      ASSERT_LE(desired_joint_states_.accelerations[0], 0.0);  // should decelerate

      Integrate(period.seconds());

      ASSERT_LE(current_joint_states_.positions[0], 5.0);  // below joint limit
    }
  }
}

TEST_F(SimpleJointLimiterTest, when_acceleration_exceeded_expect_acc_enforced)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 50000000);

    // desired acceleration exceeds
    current_joint_states_.positions[0] = 0.0;
    current_joint_states_.velocities[0] = 0.0;
    desired_joint_states_.velocities[0] = 0.5;  // leads to acc > max acc
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      0.25,                                // vel limited max acc * dt
      5.0                                  // acc max acc
    );
  }
}

TEST_F(SimpleJointLimiterTest, when_deceleration_exceeded_expect_dec_enforced)
{
  SetupNode("simple_joint_limiter");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 50000000);

    // desired deceleration exceeds
    current_joint_states_.positions[0] = 0.0;
    current_joint_states_.velocities[0] = 1.0;
    desired_joint_states_.velocities[0] = 0.5;  // leads to acc > -max dec
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      0.625,                               // vel limited by vel-max dec * dt
      -7.5                                 // acc limited by -max dec
    );
  }
}

TEST_F(SimpleJointLimiterTest, when_deceleration_exceeded_with_no_maxdec_expect_acc_enforced)
{
  SetupNode("simple_joint_limiter_nodeclimit");
  Load();

  if (joint_limiter_)
  {
    Init();
    Configure();

    rclcpp::Duration period(0, 50000000);

    // desired deceleration exceeds
    current_joint_states_.positions[0] = 0.0;
    current_joint_states_.velocities[0] = 1.0;
    desired_joint_states_.velocities[0] = 0.5;  // leads to acc > -max acc
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      0.75,                                // vel limited by vel-max acc * dt
      -5.0                                 // acc limited to -max acc
    );
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
