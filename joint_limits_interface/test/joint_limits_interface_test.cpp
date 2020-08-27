///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#include <string>
#include <gtest/gtest.h>
#include <rclcpp/logging.hpp>
#include <joint_limits_interface/joint_limits_interface.hpp>

using std::string;
using namespace hardware_interface;
using namespace joint_limits_interface;

// Floating-point value comparison threshold
const double EPS = 1e-12;

TEST(SaturateTest, Saturate)
{
  using namespace joint_limits_interface::internal;
  const double min = -1.0;
  const double max =  2.0;
  double val;

  val = -0.5;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = 0.5;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = -1.0;
  EXPECT_NEAR(val, saturate(min, min, max), EPS);

  val = -2.0;
  EXPECT_NEAR(min, saturate(val, min, max), EPS);

  val = 2.0;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = 3.0;
  EXPECT_NEAR(max, saturate(val, min, max), EPS);
}


class JointLimitsTest
{
public:
  JointLimitsTest()
    : pos(0.0), vel(0.0), eff(0.0), cmd(0.0),
      name("joint_name"),
      period(0.1),
      cmd_handle(JointStateHandle(name, &pos, &vel, &eff), &cmd)
  {
    limits.has_position_limits = true;
    limits.min_position = -1.0;
    limits.max_position =  1.0;

    limits.has_velocity_limits = true;
    limits.max_velocity = 2.0;

    limits.has_effort_limits = true;
    limits.max_effort = 8.0;

    soft_limits.min_position = -0.8;
    soft_limits.max_position =  0.8;
    soft_limits.k_position = 20.0;
    soft_limits.k_velocity = 40.0; // TODO: Tune value
  }

protected:
  double pos, vel, eff, cmd;
  string name;
  ros::Duration period;
  JointHandle cmd_handle;
  JointLimits limits;
  SoftJointLimits soft_limits;
};

class JointLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST_F(JointLimitsHandleTest, AssertionTriggering)
{
  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(PositionJointSoftLimitsHandle().enforceLimits(period), ".*");
  EXPECT_DEATH(EffortJointSoftLimitsHandle().enforceLimits(period), ".*");
  EXPECT_DEATH(VelocityJointSaturationHandle().enforceLimits(period), ".*");

  // Negative period should trigger an assertion
  EXPECT_DEATH(PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits).enforceLimits(ros::Duration(-0.1)), ".*");

  limits.has_acceleration_limits = true;
  EXPECT_DEATH(VelocityJointSaturationHandle(cmd_handle, limits).enforceLimits(ros::Duration(-0.1)), ".*");
}
#endif // NDEBUG

TEST_F(JointLimitsHandleTest, HandleConstruction)
{
  {
    JointLimits limits_bad;
    EXPECT_THROW(PositionJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits), JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be descriptive
    try {PositionJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits);}
    catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
  }

  {
    JointLimits limits_bad;
    limits_bad.has_effort_limits = true;
    EXPECT_THROW(EffortJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits), JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be descriptive
    try {EffortJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits);}
    catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
  }

  {
    JointLimits limits_bad;
    limits_bad.has_velocity_limits = true;
    EXPECT_THROW(EffortJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits), JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be descriptive
    try {EffortJointSoftLimitsHandle(cmd_handle, limits_bad, soft_limits);}
    catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
  }

  {
    JointLimits limits_bad;
    EXPECT_THROW(VelocityJointSaturationHandle(cmd_handle, limits_bad), JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be descriptive
    try {VelocityJointSaturationHandle(cmd_handle, limits_bad);}
    catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
  }

  EXPECT_NO_THROW(PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(EffortJointSoftLimitsHandle(cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(VelocityJointSaturationHandle(cmd_handle, limits));
}

class PositionJointSoftLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(PositionJointSoftLimitsHandleTest, EnforceVelocityBounds)
{
  // Test setup
  const double max_increment = period.toSec() * limits.max_velocity;
  pos = 0.0;

  double cmd;

  // Move slower than maximum velocity
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = max_increment / 2.0;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);
  }
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = -max_increment / 2.0;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);
  }

  // Move at maximum velocity
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = max_increment;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);
  }
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = -max_increment;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);
  }

  // Try to move faster than the maximum velocity, enforce velocity limits
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = 2.0 * max_increment;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(max_increment, cmd_handle.getCommand(), EPS);
  }
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);
    cmd = -2.0 * max_increment;
    cmd_handle.setCommand(cmd);
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(-max_increment, cmd_handle.getCommand(), EPS);
  }
}

// This is a black box test and does not verify against random precomputed values, but rather that the expected
// qualitative behavior is honored
TEST_F(PositionJointSoftLimitsHandleTest, EnforcePositionBounds)
{
  // Test setup
  const double workspace_center = (limits.min_position + limits.max_position) / 2.0;

  // Current position == upper soft limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero max velocity)
    pos = soft_limits.max_position;
    cmd_handle.setCommand(limits.max_position); // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd_handle.getPosition(), cmd_handle.getCommand(), EPS);

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);   // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position == lower soft limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero min velocity)
    pos = soft_limits.min_position;
    cmd_handle.setCommand(limits.min_position); // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd_handle.getPosition(), cmd_handle.getCommand(), EPS);

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);   // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position > upper soft limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (negative max velocity)
    pos = (soft_limits.max_position + limits.max_position) / 2.0; // Halfway between soft and hard limit
    cmd_handle.setCommand(limits.max_position);           // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);             // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position < lower soft limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (positive min velocity)
    pos = (soft_limits.min_position + limits.min_position) / 2.0; // Halfway between soft and hard limit
    cmd_handle.setCommand(limits.min_position);           // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);             // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }
}

TEST_F(PositionJointSoftLimitsHandleTest, PathologicalSoftBounds)
{
  // Safety limits are past the hard limits
  soft_limits.min_position = limits.min_position * (1.0 - 0.5 * limits.min_position / std::abs(limits.min_position));
  soft_limits.max_position = limits.max_position * (1.0 + 0.5 * limits.max_position / std::abs(limits.max_position));

  // Current position == higher hard limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Hit hard limit
    pos = limits.max_position;                        // On hard limit
    cmd_handle.setCommand(2.0 * limits.max_position); // Way beyond hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(limits.max_position, cmd_handle.getCommand(), EPS);
  }

  // Current position == lower hard limit
  {
    PositionJointSoftLimitsHandle limits_handle(cmd_handle, limits, soft_limits);

    // Hit hard limit
    pos = limits.min_position;                        // On hard limit
    cmd_handle.setCommand(2.0 * limits.min_position); // Way beyond hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(limits.min_position, cmd_handle.getCommand(), EPS);
  }
}

class VelocityJointSaturationHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(VelocityJointSaturationHandleTest, EnforceVelocityBounds)
{
  // Test setup
  VelocityJointSaturationHandle limits_handle(cmd_handle, limits);

  pos = 0.0;
  double cmd;

  // Velocity within bounds
  cmd = limits.max_velocity / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -limits.max_velocity / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Velocity at bounds
  cmd = limits.max_velocity;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -limits.max_velocity;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Velocity beyond bounds
  cmd = 2.0 * limits.max_velocity;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(limits.max_velocity, cmd_handle.getCommand(), EPS);

  cmd = -2.0 * limits.max_velocity;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.getCommand(), EPS);
}

TEST_F(VelocityJointSaturationHandleTest, EnforceAccelerationBounds)
{
  // Test setup
  limits.has_acceleration_limits = true;
  limits.max_acceleration = limits.max_velocity / period.toSec();
  VelocityJointSaturationHandle limits_handle(cmd_handle, limits);

  pos = 0.0;
  double cmd;
  const ros::Duration long_enough(1000.0); // An arbitrarily long time, sufficient to suppress acceleration limits

  // Positive velocity
  cmd_handle.setCommand(limits.max_velocity / 2.0); // register last command
  limits_handle.enforceLimits(long_enough); // make sure the prev_cmd is registered
                                            // without triggering the acceleration limits

  cmd = limits.max_velocity * 2.0; // Try to go beyond +max velocity
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(limits.max_velocity, cmd_handle.getCommand(), EPS); // Max velocity bounded by velocity limit

  cmd_handle.setCommand(limits.max_velocity / 2.0); // register last command
  limits_handle.enforceLimits(long_enough); // make sure the prev_cmd is registered
                                            // without triggering the acceleration limits

  cmd = -limits.max_velocity * 2.0; // Try to go beyond -max velocity
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-limits.max_velocity / 2.0, cmd_handle.getCommand(), EPS); // Max velocity bounded by acceleration limit

  // Negative velocity
  cmd_handle.setCommand(-limits.max_velocity / 2.0); // register last command
  limits_handle.enforceLimits(long_enough); // make sure the prev_cmd is registered
                                            // without triggering the acceleration limits

  cmd = limits.max_velocity * 2.0; // Try to go beyond +max velocity
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(limits.max_velocity / 2.0, cmd_handle.getCommand(), EPS); // Max velocity bounded by acceleration limit

  cmd_handle.setCommand(-limits.max_velocity / 2.0); // register last command
  limits_handle.enforceLimits(long_enough); // make sure the prev_cmd is registered
                                            // without triggering the acceleration limits

  cmd = -limits.max_velocity * 2.0; // Try to go beyond -max velocity
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.getCommand(), EPS); // Max velocity bounded by velocity limit
}

class JointLimitsInterfaceTest :public JointLimitsTest, public ::testing::Test
{
public:
  JointLimitsInterfaceTest()
    : JointLimitsTest(),
      pos2(0.0), vel2(0.0), eff2(0.0), cmd2(0.0),
      name2("joint2_name"),
      cmd_handle2(JointStateHandle(name2, &pos2, &vel2, &eff2), &cmd2)
  {}

protected:
  double pos2, vel2, eff2, cmd2;
  string name2;
  JointHandle cmd_handle2;
};

TEST_F(JointLimitsInterfaceTest, InterfaceRegistration)
{
  // Populate interface
  PositionJointSoftLimitsHandle limits_handle1(cmd_handle, limits, soft_limits);
  PositionJointSoftLimitsHandle limits_handle2(cmd_handle2, limits, soft_limits);

  PositionJointSoftLimitsInterface iface;
  iface.registerHandle(limits_handle1);
  iface.registerHandle(limits_handle2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name));
  EXPECT_NO_THROW(iface.getHandle(name2));

  PositionJointSoftLimitsHandle h1_tmp = iface.getHandle(name);
  EXPECT_EQ(name, h1_tmp.getName());

  PositionJointSoftLimitsHandle h2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, h2_tmp.getName());

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base class)
  try {iface.getHandle("unknown_name");}
  catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}

  // Enforce limits of all managed joints
  pos = pos2 = (soft_limits.max_position + limits.max_position) / 2.0; // Halfway between soft and hard limit
  cmd_handle.setCommand(limits.max_position);                               // Try to get closer to the hard limit
  cmd_handle2.setCommand(limits.max_position);
  iface.enforceLimits(period);
  EXPECT_GT(cmd_handle.getPosition(),  cmd_handle.getCommand());
  EXPECT_GT(cmd_handle2.getPosition(), cmd_handle2.getCommand());
}

TEST_F(JointLimitsHandleTest, ResetSaturationInterface)
{
  // Populate interface
  PositionJointSaturationHandle limits_handle1(cmd_handle, limits);

  PositionJointSaturationInterface iface;
  iface.registerHandle(limits_handle1);

  iface.enforceLimits(period); // initialize limit handles

  const double max_increment = period.toSec() * limits.max_velocity;

  cmd_handle.setCommand(limits.max_position);
  iface.enforceLimits(period);

  EXPECT_NEAR(cmd_handle.getCommand(),  max_increment, EPS);

  iface.reset();
  pos = limits.max_position;
  cmd_handle.setCommand(limits.max_position);
  iface.enforceLimits(period);

  EXPECT_NEAR(cmd_handle.getCommand(),  limits.max_position, EPS);

}


TEST_F(JointLimitsHandleTest, ResetSoftLimitsInterface)
{
  // Populate interface
  PositionJointSoftLimitsHandle limits_handle1(cmd_handle, limits, soft_limits);

  PositionJointSoftLimitsInterface iface;
  iface.registerHandle(limits_handle1);

  iface.enforceLimits(period); // initialize limit handles

  const double max_increment = period.toSec() * limits.max_velocity;

  cmd_handle.setCommand(limits.max_position);
  iface.enforceLimits(period);

  EXPECT_NEAR(cmd_handle.getCommand(),  max_increment, EPS);

  iface.reset();
  pos = limits.max_position;
  cmd_handle.setCommand(soft_limits.max_position);
  iface.enforceLimits(period);

  EXPECT_NEAR(cmd_handle.getCommand(),  soft_limits.max_position, EPS);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
