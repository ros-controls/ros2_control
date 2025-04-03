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

#ifndef TEST_JOINT_LIMITER_HPP_
#define TEST_JOINT_LIMITER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"

const double COMMON_THRESHOLD = 1.0e-6;

using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>;

class JointLimiterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_name_ = ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  void SetupNode(const std::string node_name = "")
  {
    if (!node_name.empty())
    {
      node_name_ = node_name;
    }
    node_ = std::make_shared<rclcpp::Node>(node_name_);
  }

  bool Load()
  {
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_.createUnmanagedInstance(joint_limiter_type_));
    return joint_limiter_ != nullptr;
  }

  bool Init(const std::string & joint_name = "foo_joint")
  {
    joint_names_ = {joint_name};
    num_joints_ = joint_names_.size();
    last_commanded_state_.joint_name = joint_name;
    last_commanded_state_.position = 0.0;
    last_commanded_state_.velocity = 0.0;
    last_commanded_state_.acceleration = 0.0;
    last_commanded_state_.effort = 0.0;
    desired_state_ = last_commanded_state_;
    actual_state_ = last_commanded_state_;
    return joint_limiter_->init(joint_names_, node_);
  }

  bool Init(const joint_limits::JointLimits & limits, const std::string & joint_name = "foo_joint")
  {
    joint_names_ = {joint_name};
    num_joints_ = joint_names_.size();
    last_commanded_state_.joint_name = joint_name;
    last_commanded_state_.position = 0.0;
    last_commanded_state_.velocity = 0.0;
    last_commanded_state_.acceleration = 0.0;
    last_commanded_state_.effort = 0.0;
    desired_state_ = last_commanded_state_;
    actual_state_ = last_commanded_state_;
    return joint_limiter_->init(
      joint_names_, {limits}, std::vector<joint_limits::SoftJointLimits>(), nullptr,
      node_->get_node_logging_interface());
  }

  bool Configure() { return joint_limiter_->configure(last_commanded_state_); }

  void Integrate(double dt)
  {
    actual_state_.position = actual_state_.position.value() + desired_state_.velocity.value() * dt +
                             0.5 * desired_state_.acceleration.value() * dt * dt;
    actual_state_.velocity =
      actual_state_.velocity.value() + desired_state_.acceleration.value() * dt;
  }

  explicit JointLimiterTest(const std::string & joint_limiter_type)
  : joint_limiter_type_(joint_limiter_type),
    joint_limiter_loader_(
      "joint_limits",
      "joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>")
  {
  }

  virtual ~JointLimiterTest() {}

  void TearDown() override { node_.reset(); }

protected:
  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  std::unique_ptr<JointLimiter> joint_limiter_;
  std::string joint_limiter_type_;
  pluginlib::ClassLoader<JointLimiter> joint_limiter_loader_;

  joint_limits::JointControlInterfacesData last_commanded_state_;
  joint_limits::JointControlInterfacesData desired_state_;
  joint_limits::JointControlInterfacesData actual_state_;
};

class JointSaturationLimiterTest : public JointLimiterTest
{
public:
  JointSaturationLimiterTest() : JointLimiterTest("joint_limits/JointInterfacesSaturationLimiter")
  {
  }
};

class JointSoftLimiterTest : public JointLimiterTest
{
public:
  JointSoftLimiterTest() : JointLimiterTest("joint_limits/JointInterfacesSoftLimiter") {}

  bool Init(
    const joint_limits::JointLimits & limits, const joint_limits::SoftJointLimits & soft_limits,
    const std::string & joint_name = "foo_joint")
  {
    joint_names_ = {joint_name};
    num_joints_ = joint_names_.size();
    last_commanded_state_.joint_name = joint_name;
    last_commanded_state_.position = 0.0;
    last_commanded_state_.velocity = 0.0;
    last_commanded_state_.acceleration = 0.0;
    last_commanded_state_.effort = 0.0;
    desired_state_ = last_commanded_state_;
    actual_state_ = last_commanded_state_;
    return joint_limiter_->init(
      joint_names_, {limits}, {soft_limits}, nullptr, node_->get_node_logging_interface());
  }
};

#endif  // TEST_JOINT_LIMITER_HPP_
