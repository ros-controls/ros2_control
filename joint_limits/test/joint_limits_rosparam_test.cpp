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

#include <memory>

#include "gtest/gtest.h"

#include "joint_limits/joint_limits_rosparam.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class JointLimitsRosParamTest : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true);

    node_ = rclcpp::Node::make_shared("JointLimitsRosparamTestNode", node_options);
  }

  void TearDown() { node_.reset(); }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(JointLimitsRosParamTest, parse_joint_limits)
{
  // Invalid specification
  {
    joint_limits::JointLimits limits;

    // test default values
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_TRUE(std::isnan(limits.min_position));
    EXPECT_TRUE(std::isnan(limits.max_position));
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_TRUE(std::isnan(limits.max_velocity));
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_TRUE(std::isnan(limits.max_acceleration));
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_TRUE(std::isnan(limits.max_jerk));
    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(std::isnan(limits.max_effort));
    EXPECT_FALSE(limits.angle_wraparound);

    // try to read limits for not-existing joints
    EXPECT_FALSE(get_joint_limits(
      "bad_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));
    EXPECT_FALSE(get_joint_limits(
      "unknown_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

    // default values should not change
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_TRUE(std::isnan(limits.min_position));
    EXPECT_TRUE(std::isnan(limits.max_position));
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_TRUE(std::isnan(limits.max_velocity));
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_TRUE(std::isnan(limits.max_acceleration));
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_TRUE(std::isnan(limits.max_jerk));
    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(std::isnan(limits.max_effort));
    EXPECT_FALSE(limits.angle_wraparound);
  }

  // Get full specification from parameter server
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "foo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

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

    // parameters is 'true', but because there are position limits it is ignored
    EXPECT_FALSE(limits.angle_wraparound);
  }

  // Specifying flags but not values should set nothing
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "yinfoo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Specifying values but not flags should set nothing
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "yangfoo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Disable already set values
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "foo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));
    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_TRUE(limits.has_acceleration_limits);
    EXPECT_TRUE(limits.has_jerk_limits);
    EXPECT_TRUE(limits.has_effort_limits);

    EXPECT_TRUE(get_joint_limits(
      "antifoo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(limits.angle_wraparound);
  }

  // Incomplete position limits specification does not get loaded
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "baz_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_TRUE(std::isnan(limits.min_position));
    EXPECT_TRUE(std::isnan(limits.max_position));
  }

  // Override only one field, leave all others unchanged
  {
    joint_limits::JointLimits limits;
    EXPECT_TRUE(get_joint_limits(
      "bar_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_TRUE(std::isnan(limits.min_position));
    EXPECT_TRUE(std::isnan(limits.max_position));

    // Max velocity is overridden
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_EQ(2.0, limits.max_velocity);

    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_TRUE(std::isnan(limits.max_acceleration));

    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_TRUE(std::isnan(limits.max_jerk));

    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(std::isnan(limits.max_effort));
  }
}

TEST_F(JointLimitsRosParamTest, parse_soft_joint_limits)
{
  // Invalid specification
  {
    joint_limits::SoftJointLimits soft_limits;

    // test default values
    EXPECT_TRUE(std::isnan(soft_limits.min_position));
    EXPECT_TRUE(std::isnan(soft_limits.max_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_velocity));

    // try to read limits for not-existing joints
    EXPECT_FALSE(get_joint_limits(
      "bad_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      soft_limits));
    EXPECT_FALSE(get_joint_limits(
      "unknown_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      soft_limits));

    // default values should not change
    EXPECT_TRUE(std::isnan(soft_limits.min_position));
    EXPECT_TRUE(std::isnan(soft_limits.max_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_velocity));
  }

  // Get full specification from parameter server
  {
    joint_limits::SoftJointLimits soft_limits;
    EXPECT_TRUE(get_joint_limits(
      "foo_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      soft_limits));

    EXPECT_EQ(10.0, soft_limits.k_position);
    EXPECT_EQ(20.0, soft_limits.k_velocity);
    EXPECT_EQ(0.1, soft_limits.min_position);
    EXPECT_EQ(0.9, soft_limits.max_position);
  }

  // Skip parsing soft limits if has_soft_limits is false
  {
    joint_limits::SoftJointLimits soft_limits;
    EXPECT_FALSE(get_joint_limits(
      "foobar_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      soft_limits));
    EXPECT_TRUE(std::isnan(soft_limits.min_position));
    EXPECT_TRUE(std::isnan(soft_limits.max_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_velocity));
  }

  // Incomplete soft limits specification does not get loaded
  {
    joint_limits::SoftJointLimits soft_limits;
    EXPECT_FALSE(get_joint_limits(
      "barbaz_joint", node_->get_node_parameters_interface(), node_->get_node_logging_interface(),
      soft_limits));
    EXPECT_TRUE(std::isnan(soft_limits.min_position));
    EXPECT_TRUE(std::isnan(soft_limits.max_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_position));
    EXPECT_TRUE(std::isnan(soft_limits.k_velocity));
  }
}

class JointLimitsUndeclaredRosParamTest : public ::testing::Test
{
public:
  void SetUp() { node_ = rclcpp::Node::make_shared("JointLimitsRosparamTestNode"); }

  void TearDown() { node_.reset(); }

protected:
  rclcpp::Node::SharedPtr node_;
};

class JointLimitsLifecycleNodeUndeclaredRosParamTest : public ::testing::Test
{
public:
  void SetUp()
  {
    lifecycle_node_ = rclcpp_lifecycle::LifecycleNode::make_shared("JointLimitsRosparamTestNode");
  }

  void TearDown() { lifecycle_node_.reset(); }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
};

TEST_F(JointLimitsUndeclaredRosParamTest, parse_declared_joint_limits_node)
{
  // Get full specification from parameter server - no need to test logic
  {
    joint_limits::JointLimits limits;
    // try to read limits for not-existing joints
    EXPECT_FALSE(get_joint_limits("bad_joint", node_, limits));
    EXPECT_FALSE(get_joint_limits("unknown_joint", node_, limits));

    // try to read existing but undeclared joint
    EXPECT_FALSE(get_joint_limits("foo_joint", node_, limits));

    // declare parameters
    EXPECT_TRUE(joint_limits::declare_parameters("foo_joint", node_));

    // now should be successful
    EXPECT_TRUE(get_joint_limits("foo_joint", node_, limits));

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

    // parameters is 'true', but because there are position limits it is ignored
    EXPECT_FALSE(limits.angle_wraparound);
  }
}

TEST_F(JointLimitsLifecycleNodeUndeclaredRosParamTest, parse_declared_joint_limits_lifecycle_node)
{
  // Get full specification from parameter server - no need to test logic
  {
    joint_limits::JointLimits limits;
    // try to read limits for not-existing joints
    EXPECT_FALSE(get_joint_limits("bad_joint", lifecycle_node_, limits));
    EXPECT_FALSE(get_joint_limits("unknown_joint", lifecycle_node_, limits));

    // try to read existing but undeclared joint
    EXPECT_FALSE(get_joint_limits("foo_joint", lifecycle_node_, limits));

    // declare parameters
    EXPECT_TRUE(joint_limits::declare_parameters("foo_joint", lifecycle_node_));

    // now should be successful
    EXPECT_TRUE(get_joint_limits("foo_joint", lifecycle_node_, limits));

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

    // parameters is 'true', but because there are position limits it is ignored
    EXPECT_FALSE(limits.angle_wraparound);
  }
}

TEST_F(JointLimitsUndeclaredRosParamTest, parse_declared_soft_joint_limits_node)
{
  // Get full specification from parameter server - no need to test logic
  {
    joint_limits::SoftJointLimits soft_limits;
    // try to read existing but undeclared joint
    EXPECT_FALSE(get_joint_limits("foo_joint", node_, soft_limits));

    // declare parameters
    EXPECT_TRUE(joint_limits::declare_parameters("foo_joint", node_));

    // now should be successful
    EXPECT_TRUE(get_joint_limits("foo_joint", node_, soft_limits));

    EXPECT_EQ(10.0, soft_limits.k_position);
    EXPECT_EQ(20.0, soft_limits.k_velocity);
    EXPECT_EQ(0.1, soft_limits.min_position);
    EXPECT_EQ(0.9, soft_limits.max_position);
  }
}

TEST_F(
  JointLimitsLifecycleNodeUndeclaredRosParamTest, parse_declared_soft_joint_limits_lifecycle_node)
{
  // Get full specification from parameter server - no need to test logic
  {
    joint_limits::SoftJointLimits soft_limits;
    // try to read existing but undeclared joint
    EXPECT_FALSE(get_joint_limits("foo_joint", lifecycle_node_, soft_limits));

    // declare parameters
    EXPECT_TRUE(joint_limits::declare_parameters("foo_joint", lifecycle_node_));

    // now should be successful
    EXPECT_TRUE(get_joint_limits("foo_joint", lifecycle_node_, soft_limits));

    EXPECT_EQ(10.0, soft_limits.k_position);
    EXPECT_EQ(20.0, soft_limits.k_velocity);
    EXPECT_EQ(0.1, soft_limits.min_position);
    EXPECT_EQ(0.9, soft_limits.max_position);
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
