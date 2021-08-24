// Copyright 2020 PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <joint_limits_interface/joint_limits_interface.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcppmath/clamp.hpp>

#include <memory>
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
  : pos(0.0),
    vel(0.0),
    eff(0.0),
    cmd(0.0),
    name("joint_name"),
    period(0, 100000000),
    cmd_handle(hardware_interface::JointHandle(name, "position_command", &cmd)),
    pos_handle(hardware_interface::JointHandle(name, hardware_interface::HW_IF_POSITION, &pos)),
    vel_handle(hardware_interface::JointHandle(name, hardware_interface::HW_IF_VELOCITY, &vel)),
    eff_handle(hardware_interface::JointHandle(name, hardware_interface::HW_IF_EFFORT, &eff))
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
  hardware_interface::JointHandle cmd_handle;
  hardware_interface::JointHandle pos_handle, vel_handle, eff_handle;
  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;
};

class JointLimitsHandleTest : public JointLimitsTest, public ::testing::Test
{
};

TEST_F(JointLimitsHandleTest, HandleConstruction)
{
  {
    joint_limits_interface::JointLimits limits_bad;
    EXPECT_THROW(
      joint_limits_interface::PositionJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits),
      joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should be
    // descriptive
    try
    {
      joint_limits_interface::PositionJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits);
    }
    catch (const joint_limits_interface::JointLimitsInterfaceException & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    limits_bad.has_effort_limits = true;
    EXPECT_THROW(
      joint_limits_interface::EffortJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits),
      joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection,
    // but exception message should be descriptive
    try
    {
      joint_limits_interface::EffortJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits);
    }
    catch (const joint_limits_interface::JointLimitsInterfaceException & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    limits_bad.has_velocity_limits = true;
    EXPECT_THROW(
      joint_limits_interface::EffortJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits),
      joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should
    // be descriptive
    try
    {
      joint_limits_interface::EffortJointSoftLimitsHandle(
        pos_handle, cmd_handle, limits_bad, soft_limits);
    }
    catch (const joint_limits_interface::JointLimitsInterfaceException & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  {
    joint_limits_interface::JointLimits limits_bad;
    EXPECT_THROW(
      joint_limits_interface::VelocityJointSaturationHandle(pos_handle, cmd_handle, limits_bad),
      joint_limits_interface::JointLimitsInterfaceException);

    // Print error messages. Requires manual output inspection, but exception message should
    // be descriptive
    try
    {
      joint_limits_interface::VelocityJointSaturationHandle(pos_handle, cmd_handle, limits_bad);
    }
    catch (const joint_limits_interface::JointLimitsInterfaceException & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("joint_limits_interface_test"), "%s", e.what());
    }
  }

  EXPECT_NO_THROW(joint_limits_interface::PositionJointSoftLimitsHandle(
    pos_handle, cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(joint_limits_interface::EffortJointSoftLimitsHandle(
    pos_handle, cmd_handle, limits, soft_limits));
  EXPECT_NO_THROW(
    joint_limits_interface::VelocityJointSaturationHandle(pos_handle, cmd_handle, limits));
}

class PositionJointSoftLimitsHandleTest : public JointLimitsTest, public ::testing::Test
{
};

TEST_F(PositionJointSoftLimitsHandleTest, EnforceVelocityBounds)
{
  // Test setup
  const double max_increment = period.seconds() * limits.max_velocity;
  pos = 0.0;

  double cmd;

  // Move slower than maximum velocity
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = max_increment / 2.0;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = -max_increment / 2.0;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);
  }

  // Move at maximum velocity
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = max_increment;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = -max_increment;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);
  }

  // Try to move faster than the maximum velocity, enforce velocity limits
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = 2.0 * max_increment;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(max_increment, cmd_handle.get_value(), EPS);
  }
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);
    cmd = -2.0 * max_increment;
    cmd_handle.set_value(cmd);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(-max_increment, cmd_handle.get_value(), EPS);
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
      pos_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero max velocity)
    pos = soft_limits.max_position;
    // Try to get closer to the hard limit
    cmd_handle.set_value(limits.max_position);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(pos_handle.get_value(), cmd_handle.get_value(), EPS);

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_value(workspace_center);
    limits_handle.enforce_limits(period);
    EXPECT_GT(pos_handle.get_value(), cmd_handle.get_value());
  }

  // Current position == lower soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (zero min velocity)
    pos = soft_limits.min_position;
    // Try to get closer to the hard limit
    cmd_handle.set_value(limits.min_position);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(pos_handle.get_value(), cmd_handle.get_value(), EPS);

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_value(workspace_center);
    limits_handle.enforce_limits(period);
    EXPECT_LT(pos_handle.get_value(), cmd_handle.get_value());
  }

  // Current position > upper soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (negative max velocity)
    // Halfway between soft and hard limit
    pos = (soft_limits.max_position + limits.max_position) / 2.0;
    // Try to get closer to the hard limit
    cmd_handle.set_value(limits.max_position);
    limits_handle.enforce_limits(period);
    EXPECT_GT(pos_handle.get_value(), cmd_handle.get_value());

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_value(workspace_center);
    limits_handle.enforce_limits(period);
    EXPECT_GT(pos_handle.get_value(), cmd_handle.get_value());
  }

  // Current position < lower soft limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);

    // Can't get any closer to hard limit (positive min velocity)
    // Halfway between soft and hard limit
    pos = (soft_limits.min_position + limits.min_position) / 2.0;
    // Try to get closer to the hard limit
    cmd_handle.set_value(limits.min_position);
    limits_handle.enforce_limits(period);
    EXPECT_LT(pos_handle.get_value(), cmd_handle.get_value());

    // OK to move away from hard limit
    // Try to go to workspace center
    cmd_handle.set_value(workspace_center);
    limits_handle.enforce_limits(period);
    EXPECT_LT(pos_handle.get_value(), cmd_handle.get_value());
  }
}

TEST_F(PositionJointSoftLimitsHandleTest, PathologicalSoftBounds)
{
  // Safety limits are past the hard limits
  soft_limits.min_position =
    limits.min_position * (1.0 - 0.5 * limits.min_position / std::abs(limits.min_position));
  soft_limits.max_position =
    limits.max_position * (1.0 + 0.5 * limits.max_position / std::abs(limits.max_position));

  // Current position == higher hard limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);

    // Hit hard limit
    // On hard limit
    pos = limits.max_position;
    // Way beyond hard limit
    cmd_handle.set_value(2.0 * limits.max_position);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(limits.max_position, cmd_handle.get_value(), EPS);
  }

  // Current position == lower hard limit
  {
    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
      pos_handle, cmd_handle, limits, soft_limits);

    // Hit hard limit
    // On hard limit
    pos = limits.min_position;
    // Way beyond hard limit
    cmd_handle.set_value(2.0 * limits.min_position);
    limits_handle.enforce_limits(period);
    EXPECT_NEAR(limits.min_position, cmd_handle.get_value(), EPS);
  }
}

class VelocityJointSaturationHandleTest : public JointLimitsTest, public ::testing::Test
{
};

TEST_F(VelocityJointSaturationHandleTest, EnforceVelocityBounds)
{
  // Test setup
  joint_limits_interface::VelocityJointSaturationHandle limits_handle(
    pos_handle, cmd_handle, limits);

  pos = 0.0;
  double cmd;

  // Velocity within bounds
  cmd = limits.max_velocity / 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);

  cmd = -limits.max_velocity / 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);

  // Velocity at bounds
  cmd = limits.max_velocity;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);

  cmd = -limits.max_velocity;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(cmd, cmd_handle.get_value(), EPS);

  // Velocity beyond bounds
  cmd = 2.0 * limits.max_velocity;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(limits.max_velocity, cmd_handle.get_value(), EPS);

  cmd = -2.0 * limits.max_velocity;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.get_value(), EPS);
}

TEST_F(VelocityJointSaturationHandleTest, EnforceAccelerationBounds)
{
  // Test setup
  limits.has_acceleration_limits = true;
  limits.max_acceleration = limits.max_velocity / period.seconds();
  joint_limits_interface::VelocityJointSaturationHandle limits_handle(
    pos_handle, cmd_handle, limits);

  pos = 0.0;
  double cmd;
  // An arbitrarily long time, sufficient to suppress acceleration limits
  const rclcpp::Duration long_enough(1000, 0);

  // Positive velocity
  // register last command
  cmd_handle.set_value(limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforce_limits(long_enough);

  // Try to go beyond +max velocity
  cmd = limits.max_velocity * 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  // Max velocity bounded by velocity limit
  EXPECT_NEAR(limits.max_velocity, cmd_handle.get_value(), EPS);

  // register last command
  cmd_handle.set_value(limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforce_limits(long_enough);

  // Try to go beyond -max velocity
  cmd = -limits.max_velocity * 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  // Max velocity bounded by acceleration limit
  EXPECT_NEAR(-limits.max_velocity / 2.0, cmd_handle.get_value(), EPS);

  // Negative velocity
  // register last command
  cmd_handle.set_value(-limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforce_limits(long_enough);

  // Try to go beyond +max velocity
  cmd = limits.max_velocity * 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  // Max velocity bounded by acceleration limit
  EXPECT_NEAR(limits.max_velocity / 2.0, cmd_handle.get_value(), EPS);

  // register last command
  cmd_handle.set_value(-limits.max_velocity / 2.0);
  // make sure the prev_cmd is registered
  // without triggering the acceleration limits
  limits_handle.enforce_limits(long_enough);

  // Try to go beyond -max velocity
  cmd = -limits.max_velocity * 2.0;
  cmd_handle.set_value(cmd);
  limits_handle.enforce_limits(period);
  // Max velocity bounded by velocity limit
  EXPECT_NEAR(-limits.max_velocity, cmd_handle.get_value(), EPS);
}

class JointLimitsInterfaceTest : public JointLimitsTest, public ::testing::Test
{
public:
  JointLimitsInterfaceTest()
  : JointLimitsTest(),
    pos2(0.0),
    vel2(0.0),
    eff2(0.0),
    cmd2(0.0),
    name2("joint2_name"),
    cmd2_handle(
      std::make_shared<hardware_interface::JointHandle>(name2, "position_command", &cmd2)),
    pos2_handle(std::make_shared<hardware_interface::JointHandle>(
      name2, hardware_interface::HW_IF_POSITION, &pos2)),
    vel2_handle(std::make_shared<hardware_interface::JointHandle>(
      name2, hardware_interface::HW_IF_VELOCITY, &vel2)),
    eff2_handle(std::make_shared<hardware_interface::JointHandle>(
      name2, hardware_interface::HW_IF_EFFORT, &eff2))
  {
  }

protected:
  double pos2, vel2, eff2, cmd2;
  std::string name2;
  std::shared_ptr<hardware_interface::JointHandle> cmd2_handle;
  std::shared_ptr<hardware_interface::JointHandle> pos2_handle, vel2_handle, eff2_handle;
};

// TEST_F(JointLimitsInterfaceTest, InterfaceRegistration)
// {
//   // Populate interface
//   joint_limits_interface::PositionJointSoftLimitsHandle limits_handle1(
//     pos_handle, cmd_handle, limits, soft_limits);
//   joint_limits_interface::PositionJointSoftLimitsHandle limits_handle2(
//     pos_handle2, cmd_handle2, limits, soft_limits);
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
//     ROS_ERROR("%s", e.what());
//   }
//
//   // Enforce limits of all managed joints
//   // Halfway between soft and hard limit
//   pos = pos2 = (soft_limits.max_position + limits.max_position) / 2.0;
//   // Try to get closer to the hard limit
//   cmd_handle.set_value(limits.max_position);
//   cmd_handle2.set_cmd(limits.max_position);
//   iface.enforce_limits(period);
//   EXPECT_GT(pos_handle.get_value(), cmd_handle.get_value());
//   EXPECT_GT(cmd_handle2.getPosition(), cmd_handle2.get_cmd());
// }
//
#if 0  // todo: implement the interfaces
TEST_F(JointLimitsHandleTest, ResetSaturationInterface)
{
  // Populate interface
  joint_limits_interface::PositionJointSaturationHandle limits_handle1
    (pos_handle, cmd_handle, limits);

  PositionJointSaturationInterface iface;
  iface.registerHandle(limits_handle1);

  iface.enforce_limits(period);  // initialize limit handles

  const double max_increment = period.seconds() * limits.max_velocity;

  cmd_handle.set_value(limits.max_position);
  iface.enforce_limits(period);

  EXPECT_NEAR(cmd_handle.get_value(), max_increment, EPS);

  iface.reset();
  pos = limits.max_position;
  cmd_handle.set_value(limits.max_position);
  iface.enforce_limits(period);

  EXPECT_NEAR(cmd_handle.get_value(), limits.max_position, EPS);
}
#endif

// TEST_F(JointLimitsHandleTest, ResetSoftLimitsInterface)
// {
//   // Populate interface
//   PositionJointSoftLimitsHandle limits_handle1(cmd_handle, limits, soft_limits);
//
//   PositionJointSoftLimitsInterface iface;
//   iface.registerHandle(limits_handle1);
//
//   iface.enforce_limits(period); // initialize limit handles
//
//   const double max_increment = period.seconds() * limits.max_velocity;
//
//   cmd_handle.set_value(limits.max_position);
//   iface.enforce_limits(period);
//
//   EXPECT_NEAR(cmd_handle.get_value(), max_increment, EPS);
//
//   iface.reset();
//   pos = limits.max_position;
//   cmd_handle.set_value(soft_limits.max_position);
//   iface.enforce_limits(period);
//
//   EXPECT_NEAR(cmd_handle.get_value(), soft_limits.max_position, EPS);
//
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
