// Copyright 2020 ros2_control development team
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

#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

using namespace ::testing;  // NOLINT

using hardware_interface::components::ComponentInfo;
using hardware_interface::components::Joint;
using hardware_interface::hardware_interface_status;
using hardware_interface::return_type;

TEST(PositionJointTest, position_joint_ok_test)
{
  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";
  position_joint_info.parameters["min_value"] = "-3.14";
  position_joint_info.parameters["max_value"] = "3.14";

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "ros2_control_components", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::OK);
  ASSERT_THAT(joint->get_command_interfaces(), SizeIs(1));
  EXPECT_EQ(joint->get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
  ASSERT_THAT(joint->get_state_interfaces(), SizeIs(1));
  EXPECT_EQ(joint->get_state_interfaces()[0], hardware_interface::HW_IF_POSITION);

  // Command getters and setters
  std::vector<std::string> interfaces;
  std::vector<double> input;
  input.push_back(2.1);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  input.clear();
  input.push_back(1.2);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::OK);
//
//   std::vector<double> output;
//   interfaces.clear();
//   EXPECT_EQ(joint->get_command(output, interfaces), return_type::INTERFACE_NOT_PROVIDED);
//   interfaces.push_back(hardware_interface::HW_IF_POSITION);
//   EXPECT_EQ(joint->get_command(output, interfaces), return_type::OK);
//   ASSERT_THAT(output, SizeIs(1));
//   EXPECT_EQ(output[0], 1.2);
//   interfaces.clear();
//   interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//   EXPECT_EQ(joint->get_command(output, interfaces), return_type::INTERFACE_NOT_FOUND);
//
//   input.clear();
//   EXPECT_EQ(joint->set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//   input.push_back(2.1);
//   EXPECT_EQ(joint->set_command(input), return_type::OK);
//
//   EXPECT_EQ(joint->get_command(output), return_type::OK);
//   ASSERT_THAT(output, SizeIs(1));
//   EXPECT_EQ(output[0], 2.1);
//
//   // State getters and setters
//   interfaces.clear();
//   input.clear();
//   input.push_back(2.1);
//   EXPECT_EQ(joint->set_state(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
//   interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//   EXPECT_EQ(joint->set_state(input, interfaces), return_type::INTERFACE_NOT_FOUND);
//   interfaces.push_back(hardware_interface::HW_IF_POSITION);
//   EXPECT_EQ(joint->set_state(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//   interfaces.clear();
//   interfaces.push_back(hardware_interface::HW_IF_POSITION);
//   input.clear();
//   input.push_back(1.2);
//   EXPECT_EQ(joint->set_state(input, interfaces), return_type::OK);
//
//   output.clear();
//   interfaces.clear();
//   EXPECT_EQ(joint->get_command(output, interfaces), return_type::INTERFACE_NOT_PROVIDED);
//   interfaces.push_back(hardware_interface::HW_IF_POSITION);
//   EXPECT_EQ(joint->get_state(output, interfaces), return_type::OK);
//   ASSERT_THAT(output, SizeIs(1));
//   EXPECT_EQ(output[0], 1.2);
//   interfaces.clear();
//   interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//   EXPECT_EQ(joint->get_state(output, interfaces), return_type::INTERFACE_NOT_FOUND);
//
//   input.clear();
//   EXPECT_EQ(joint->set_state(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//   input.push_back(2.1);
//   EXPECT_EQ(joint->set_state(input), return_type::OK);
//
//   EXPECT_EQ(joint->get_state(output), return_type::OK);
//   ASSERT_THAT(output, SizeIs(1));
//   EXPECT_EQ(output[0], 2.1);
//
//   // Test DummyPositionJoint
//   joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
//   joint_info.command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//   EXPECT_EQ(joint->configure(joint_info), return_type::ERROR);
}

TEST(PositionJointTest, position_joint_failure_test)
{
  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "ros2_control_components", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";
  position_joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  position_joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);

  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_TOO_MANY_INTERFACES);

  position_joint_info.command_interfaces.clear();
  position_joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  position_joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);

  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_TOO_MANY_INTERFACES);
}
