// Copyright 2013, PAL Robotics S.L. All rights reserved.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holders nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>

#include <joint_limits_interface/joint_limits_interface.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcppmath/clamp.hpp>

#include <string>


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


class JointLimitsTest
{
public:
  JointLimitsTest()
  : pos(0.0), vel(0.0), eff(0.0), cmd(0.0),
    name("joint_name"),
    period(0, 100000000),
    cmd_handle(name, &cmd),
    state_handle(name, &pos, &vel, &eff)
  {
    limits.has_position_limits = true;
    limits.min_position = -1.0;
    limits.max_position = 1.0;

    limits.has_velocity_limits = true;
    limits.max_velocity = 2.0;

    limits.has_effort_limits = true;
    limits.max_effort = 8.0;

    soft_limits.min_position = -0.8;
    soft_limits.max_position = 0.8;
    soft_limits.k_position = 20.0;
    // TODO(anyone): Tune value
    soft_limits.k_velocity = 40.0;
  }

protected:
  double pos, vel, eff, cmd;
  std::string name;
  rclcpp::Duration period;
  hardware_interface::JointCommandHandle cmd_handle;
  hardware_interface::JointStateHandle state_handle;
  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;
};

class JointLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(JointLimitsHandleTest, HandleConstruction)
{
  {
    joint_limits_interface::JointLimits limits_bad;
    EXPECT_THROW(
      joint_limits_interface::PositionJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad,
        soft_limits), joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be
    // descriptive
    try {
      joint_limits_interface::PositionJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad, soft_limits);
    } catch (const joint_limits_interface::JointLimitsInterfaceException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    limits_bad.has_effort_limits = true;
    EXPECT_THROW(
      joint_limits_interface::EffortJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad,
        soft_limits), joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection,
    // but exception message should be descriptive
    try {
      joint_limits_interface::EffortJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad, soft_limits);
    } catch (const joint_limits_interface::JointLimitsInterfaceException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    limits_bad.has_velocity_limits = true;
    EXPECT_THROW(
      joint_limits_interface::EffortJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad,
        soft_limits), joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should
    // be descriptive
    try {
      joint_limits_interface::EffortJointSoftLimitsHandle(
        state_handle, cmd_handle, limits_bad, soft_limits);
    } catch (const joint_limits_interface::JointLimitsInterfaceException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    EXPECT_THROW(
      joint_limits_interface::VelocityJointSaturationHandle(
        state_handle, cmd_handle,
        limits_bad), joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should
    // be descriptive
    try {
      joint_limits_interface::VelocityJointSaturationHandle(state_handle, cmd_handle, limits_bad);
    } catch (const joint_limits_interface::JointLimitsInterfaceException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  EXPECT_NO_THROW(
    joint_limits_interface::PositionJointSoftLimitsHandle(
      state_handle, cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(
    joint_limits_interface::EffortJointSoftLimitsHandle(
      state_handle, cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(
    joint_limits_interface::VelocityJointSaturationHandle(
      state_handle, cmd_handle, limits));
}

class PositionJointSoftLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(PositionJointSoftLimitsHandleTest, EnforceVelocityBounds)
{
  // Test setup
  const double max_increment = period.seconds() * limits.max_velocity;
  pos = 0.0;

  double cmd;

  // Move slower than maximum velocity
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = max_increment / 2.0;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = -max_increment / 2.0;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);
  }

  // Move at maximum velocity
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = max_increment;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = -max_increment;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);
  }

  // Try to move faster than the maximum velocity, enforce velocity limits
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = 2.0 * max_increment;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(max_increment, cmd_handle.get_cmd(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);
    cmd = -2.0 * max_increment;
    cmd_handle.set_cmd(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(-max_increment, cmd_handle.get_cmd(), EPS);
  }
}

// This is a black box test and does not verify against random precomputed values, but rather that
// the expected qualitative behavior is honored
TEST_F(PositionJointSoftLimitsHandleTest, EnforcePositionBounds)
{
  // Test setup
  const double workspace_center = (limits.min_position + limits.max_position) / 2.0;

  // Current position == upper soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero max velocity)
    pos = soft_limits.max_position;
    // Try to get closer to the hard limit
    cmd_handle.set_cmd(limits.max_position);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(state_handle.get_position(), cmd_handle.get_cmd(), EPS);

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_cmd(workspace_center);
    limits_handle.enforceLimits(period);
    EXPECT_GT(state_handle.get_position(), cmd_handle.get_cmd());
  }

  // Current position == lower soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero min velocity)
    pos = soft_limits.min_position;
    // Try to get closer to the hard limit
    cmd_handle.set_cmd(limits.min_position);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(state_handle.get_position(), cmd_handle.get_cmd(), EPS);

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_cmd(workspace_center);
    limits_handle.enforceLimits(period);
    EXPECT_LT(state_handle.get_position(), cmd_handle.get_cmd());
  }

  // Current position > upper soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (negative max velocity)
    // Halfway between soft and hard limit
    pos = (soft_limits.max_position + limits.max_position) / 2.0;
    // Try to get closer to the hard limit
    cmd_handle.set_cmd(limits.max_position);
    limits_handle.enforceLimits(period);
    EXPECT_GT(state_handle.get_position(), cmd_handle.get_cmd());

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_cmd(workspace_center);
    limits_handle.enforceLimits(period);
    EXPECT_GT(state_handle.get_position(), cmd_handle.get_cmd());
  }

  // Current position < lower soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (positive min velocity)
    // Halfway between soft and hard limit
    pos = (soft_limits.min_position + limits.min_position) / 2.0;
    // Try to get closer to the hard limit
    cmd_handle.set_cmd(limits.min_position);
    limits_handle.enforceLimits(period);
    EXPECT_LT(state_handle.get_position(), cmd_handle.get_cmd());

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_cmd(workspace_center);
    limits_handle.enforceLimits(period);
    EXPECT_LT(state_handle.get_position(), cmd_handle.get_cmd());
  }
}

TEST_F(PositionJointSoftLimitsHandleTest, PathologicalSoftBounds)
{
  // Safety limits are past the hard limits
  soft_limits.min_position = limits.min_position *
    (1.0 - 0.5 * limits.min_position / std::abs(limits.min_position));
  soft_limits.max_position = limits.max_position *
    (1.0 + 0.5 * limits.max_position / std::abs(limits.max_position));

  // Current position == higher hard limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Hit hard limit
    // On hard limit
    pos = limits.max_position;
    // Way beyond hard limit
    cmd_handle.set_cmd(2.0 * limits.max_position);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(limits.max_position, cmd_handle.get_cmd(), EPS);
  }

  // Current position == lower hard limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      state_handle, cmd_handle, limits, soft_limits);

    // Hit hard limit
    // On hard limit
    pos = limits.min_position;
    // Way beyond hard limit
    cmd_handle.set_cmd(2.0 * limits.min_position);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(limits.min_position, cmd_handle.get_cmd(), EPS);
  }
}

class VelocityJointSaturationHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(VelocityJointSaturationHandleTest, EnforceVelocityBounds)
{
  // Test setup
  joint_limits_interface::VelocityJointSaturationHandle limits_handle(
    state_handle, cmd_handle, limits);

  pos = 0.0;
  double cmd;

  // Velocity within bounds
  cmd = limits.max_velocity / 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);

  cmd = -limits.max_velocity / 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);

  // Velocity at bounds
  cmd = limits.max_velocity;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);

  cmd = -limits.max_velocity;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_cmd(), EPS);

  // Velocity beyond bounds
  cmd = 2.0 * limits.max_velocity;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(limits.max_velocity, cmd_handle.get_cmd(), EPS);

  cmd = -2.0 * limits.max_velocity;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.get_cmd(), EPS);
}

TEST_F(VelocityJointSaturationHandleTest, EnforceAccelerationBounds)
{
  // Test setup
  limits.has_acceleration_limits = true;
  limits.max_acceleration = limits.max_velocity / period.seconds();
  joint_limits_interface::VelocityJointSaturationHandle limits_handle(
    state_handle, cmd_handle, limits);

  pos = 0.0;
  double cmd;
  // An arbitrarily long time, sufficient to suppress acceleration limits
  const rclcpp::Duration long_enough(1000, 0);

  // Positive velocity
  // register last command
  cmd_handle.set_cmd(limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforceLimits(long_enough);

  // Try to go beyond +max velocity
  cmd = limits.max_velocity * 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  // Max velocity bounded by velocity limit
  EXPECT_NEAR(limits.max_velocity, cmd_handle.get_cmd(), EPS);

  // register last command
  cmd_handle.set_cmd(limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforceLimits(long_enough);

  // Try to go beyond -max velocity
  cmd = -limits.max_velocity * 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  // Max velocity bounded by acceleration limit
  EXPECT_NEAR(-limits.max_velocity / 2.0, cmd_handle.get_cmd(), EPS);

  // Negative velocity
  // register last command
  cmd_handle.set_cmd(-limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforceLimits(long_enough);

  // Try to go beyond +max velocity
  cmd = limits.max_velocity * 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  // Max velocity bounded by acceleration limit
  EXPECT_NEAR(limits.max_velocity / 2.0, cmd_handle.get_cmd(), EPS);

  // register last command
  cmd_handle.set_cmd(-limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforceLimits(long_enough);

  // Try to go beyond -max velocity
  cmd = -limits.max_velocity * 2.0;
  cmd_handle.set_cmd(cmd);
  limits_handle.enforceLimits(period);
  // Max velocity bounded by velocity limit
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.get_cmd(), EPS);
}

class JointLimitsInterfaceTest : public JointLimitsTest, public ::testing::Test
{
public:
  JointLimitsInterfaceTest()
  : JointLimitsTest(),
    pos2(0.0), vel2(0.0), eff2(0.0), cmd2(0.0),
    name2("joint2_name"),
    cmd_handle2(name2, &cmd2),
    state_handle2(name2, &pos2, &vel2, &eff2)
  {}

protected:
  double pos2, vel2, eff2, cmd2;
  std::string name2;
  hardware_interface::JointCommandHandle cmd_handle2;
  hardware_interface::JointStateHandle state_handle2;
};

// TEST_F(JointLimitsInterfaceTest, InterfaceRegistration)
// {
//   // Populate interface
//   joint_limits_interface::PositionJointSoftLimitsHandle limits_handle1(
//     state_handle, cmd_handle, limits, soft_limits);
//   joint_limits_interface::PositionJointSoftLimitsHandle limits_handle2(
//     state_handle2, cmd_handle2, limits, soft_limits);
//
//   joint_limits_interface::PositionJointSoftLimitsInterface iface;
//   iface.registerHandle(limits_handle1);
//   iface.registerHandle(limits_handle2);
//
//   // Get handles
//   EXPECT_NO_THROW(iface.getHandle(name));
//   EXPECT_NO_THROW(iface.getHandle(name2));
//
//   PositionJointSoftLimitsHandle h1_tmp = iface.getHandle(name);
//   EXPECT_EQ(name, h1_tmp.getName());
//
//   PositionJointSoftLimitsHandle h2_tmp = iface.getHandle(name2);
//   EXPECT_EQ(name2, h2_tmp.getName());
//
//   // Print error message
//   // Requires manual output inspection, but exception message should contain the interface name
//   // (not its base class)
//   try {
//     iface.getHandle("unknown_name");
//   } catch (const JointLimitsInterfaceException & e) {
//     ROS_ERROR_STREAM(e.what());
//   }
//
//   // Enforce limits of all managed joints
//   // Halfway between soft and hard limit
//   pos = pos2 = (soft_limits.max_position + limits.max_position) / 2.0;
//   // Try to get closer to the hard limit
//   cmd_handle.set_cmd(limits.max_position);
//   cmd_handle2.set_cmd(limits.max_position);
//   iface.enforceLimits(period);
//   EXPECT_GT(state_handle.get_position(), cmd_handle.get_cmd());
//   EXPECT_GT(cmd_handle2.getPosition(), cmd_handle2.get_cmd());
// }
//
// TEST_F(JointLimitsHandleTest, ResetSaturationInterface)
// {
//   // Populate interface
//   PositionJointSaturationHandle limits_handle1(cmd_handle, limits);
//
//   PositionJointSaturationInterface iface;
//   iface.registerHandle(limits_handle1);
//
//   iface.enforceLimits(period); // initialize limit handles
//
//   const double max_increment = period.seconds() * limits.max_velocity;
//
//   cmd_handle.set_cmd(limits.max_position);
//   iface.enforceLimits(period);
//
//   EXPECT_NEAR(cmd_handle.get_cmd(), max_increment, EPS);
//
//   iface.reset();
//   pos = limits.max_position;
//   cmd_handle.set_cmd(limits.max_position);
//   iface.enforceLimits(period);
//
//   EXPECT_NEAR(cmd_handle.get_cmd(), limits.max_position, EPS);
//
// }
//
//
// TEST_F(JointLimitsHandleTest, ResetSoftLimitsInterface)
// {
//   // Populate interface
//   PositionJointSoftLimitsHandle limits_handle1(cmd_handle, limits, soft_limits);
//
//   PositionJointSoftLimitsInterface iface;
//   iface.registerHandle(limits_handle1);
//
//   iface.enforceLimits(period); // initialize limit handles
//
//   const double max_increment = period.seconds() * limits.max_velocity;
//
//   cmd_handle.set_cmd(limits.max_position);
//   iface.enforceLimits(period);
//
//   EXPECT_NEAR(cmd_handle.get_cmd(), max_increment, EPS);
//
//   iface.reset();
//   pos = limits.max_position;
//   cmd_handle.set_cmd(soft_limits.max_position);
//   iface.enforceLimits(period);
//
//   EXPECT_NEAR(cmd_handle.get_cmd(), soft_limits.max_position, EPS);
//
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
