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

#include <joint_limits_interface/joint_limits_rosparam.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

class JointLimitsRosParamTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true);

    node_ = rclcpp::Node::make_shared("JointLimitsRosparamTestNode", node_options);
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(JointLimitsRosParamTest, GetJointLimits)
{
  // Invalid specification
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_FALSE(getJointLimits("bad_joint", node_, limits));
    EXPECT_FALSE(getJointLimits("unknown_joint", node_, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Get full specification from parameter server
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits("foo_joint", node_, limits));

    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_EQ(0.0, limits.min_position);
    EXPECT_EQ(1.0, limits.max_position);

    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_EQ(2.0, limits.max_velocity);

    EXPECT_TRUE(limits.has_acceleration_limits);
    EXPECT_EQ(5.0, limits.max_acceleration);

    EXPECT_TRUE(limits.has_jerk_limits);
    EXPECT_EQ(100.0, limits.max_jerk);

    EXPECT_TRUE(limits.has_effort_limits);
    EXPECT_EQ(20.0, limits.max_effort);

    EXPECT_FALSE(limits.angle_wraparound);
  }

  // Specifying flags but not values should set nothing
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits("yinfoo_joint", node_, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Specifying values but not flags should set nothing
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits("yangfoo_joint", node_, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Disable already set values
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits("foo_joint", node_, limits));
    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_TRUE(limits.has_acceleration_limits);
    EXPECT_TRUE(limits.has_jerk_limits);
    EXPECT_TRUE(limits.has_effort_limits);

    EXPECT_TRUE(getJointLimits("antifoo_joint", node_, limits));
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(limits.angle_wraparound);
  }

  // Incomplete position limits specification does not get loaded
  {
    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits("baz_joint", node_, limits));

    EXPECT_FALSE(limits.has_position_limits);
  }

  // Override only one field, leave all others unchanged
  {
    joint_limits_interface::JointLimits limits, limits_ref;
    EXPECT_TRUE(getJointLimits("bar_joint", node_, limits));

    EXPECT_EQ(limits_ref.has_position_limits, limits.has_position_limits);
    EXPECT_EQ(limits_ref.min_position, limits.min_position);
    EXPECT_EQ(limits_ref.max_position, limits.max_position);

    // Max velocity is overridden
    EXPECT_NE(limits_ref.has_velocity_limits, limits.has_velocity_limits);
    EXPECT_NE(limits_ref.max_velocity, limits.max_velocity);
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_EQ(2.0, limits.max_velocity);

    EXPECT_EQ(limits_ref.has_acceleration_limits, limits.has_acceleration_limits);
    EXPECT_EQ(limits_ref.max_acceleration, limits.max_acceleration);

    EXPECT_EQ(limits_ref.has_jerk_limits, limits.has_jerk_limits);
    EXPECT_EQ(limits_ref.has_jerk_limits, limits.max_jerk);

    EXPECT_EQ(limits_ref.has_effort_limits, limits.has_effort_limits);
    EXPECT_EQ(limits_ref.max_effort, limits.max_effort);
  }
}

TEST_F(JointLimitsRosParamTest, GetSoftJointLimits)
{
  // Invalid specification
  {
    joint_limits_interface::SoftJointLimits soft_limits;
    EXPECT_FALSE(getSoftJointLimits("bad_joint", node_, soft_limits));
    EXPECT_FALSE(getSoftJointLimits("unknown_joint", node_, soft_limits));
  }

  // Get full specification from parameter server
  {
    joint_limits_interface::SoftJointLimits soft_limits;
    EXPECT_TRUE(getSoftJointLimits("foo_joint", node_, soft_limits));

    EXPECT_EQ(10.0, soft_limits.k_position);
    EXPECT_EQ(20.0, soft_limits.k_velocity);
    EXPECT_EQ(0.1, soft_limits.min_position);
    EXPECT_EQ(0.9, soft_limits.max_position);
  }

  // Skip parsing soft limits if has_soft_limits is false
  {
    joint_limits_interface::SoftJointLimits soft_limits, soft_limits_ref;
    EXPECT_FALSE(getSoftJointLimits("foobar_joint", node_, soft_limits));
  }

  // Incomplete soft limits specification does not get loaded
  {
    joint_limits_interface::SoftJointLimits soft_limits, soft_limits_ref;
    EXPECT_FALSE(getSoftJointLimits("barbaz_joint", node_, soft_limits));
    EXPECT_EQ(soft_limits.k_position, soft_limits_ref.k_position);
    EXPECT_EQ(soft_limits.k_velocity, soft_limits_ref.k_velocity);
    EXPECT_EQ(soft_limits.min_position, soft_limits_ref.min_position);
    EXPECT_EQ(soft_limits.max_position, soft_limits_ref.max_position);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
