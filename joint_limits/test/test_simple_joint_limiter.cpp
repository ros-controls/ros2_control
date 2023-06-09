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

TEST_F(SimpleJointLimiterTest, load_limiter)
{
  ASSERT_NO_THROW(
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_)));
  ASSERT_NE(joint_limiter_, nullptr);
}

TEST_F(SimpleJointLimiterTest, validate_limitation)
{
  joint_limiter_ = std::unique_ptr<JointLimiter>(
    joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_));

  if (joint_limiter_)
  {
    rclcpp::Duration period(1.0, 0.0);  // 1 second

    // initialize the limiter
    std::vector<std::string> joint_names = {"foo_joint"};
    auto num_joints = joint_names.size();
    ASSERT_TRUE(joint_limiter_->init(joint_names, node_));

    last_commanded_state_.positions.resize(num_joints);
    last_commanded_state_.velocities.resize(num_joints, 0.0);
    last_commanded_state_.accelerations.resize(num_joints, 0.0);

    // no size check occurs (yet) so expect true
    ASSERT_TRUE(joint_limiter_->configure(last_commanded_state_));
    desired_joint_states_ = last_commanded_state_;
    current_joint_states_ = last_commanded_state_;

    // pos, vel, acc, dec = 1.0, 2.0, 5.0, 7.5

    desired_joint_states_.velocities.clear();
    // trigger size check with one wrong size
    ASSERT_FALSE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));
    // fix size
    desired_joint_states_.velocities.resize(num_joints, 0.0);

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

    // close to pos limit should reduce velocity and stop
    current_joint_states_.positions[0] = 4.851;
    current_joint_states_.velocities[0] = 1.5;
    desired_joint_states_.positions[0] = 4.926;
    desired_joint_states_.velocities[0] = 1.5;

    // changing to 0.05 because 1.0 sec invalidates the "small dt integration"
    period = rclcpp::Duration::from_seconds(0.05);

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

      INTEGRATE(current_joint_states_, desired_joint_states_, period.seconds());

      ASSERT_LE(current_joint_states_.positions[0], 5.0);  // below joint limit
    }

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

    // desired deceleration exceeds
    current_joint_states_.positions[0] = 0.0;
    current_joint_states_.velocities[0] = 1.0;
    desired_joint_states_.velocities[0] = 0.5;  // leads to acc > -max dec
    ASSERT_TRUE(joint_limiter_->enforce(current_joint_states_, desired_joint_states_, period));

    // check if vel and acc limits applied
    CHECK_STATE_SINGLE_JOINT(
      desired_joint_states_, 0,
      desired_joint_states_.positions[0],  // pos unchanged
      0.625,                               // vel limited by max dec * dt
      -7.5                                 // acc max dec
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
