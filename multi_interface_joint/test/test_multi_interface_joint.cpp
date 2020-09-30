// Copyright 2020 ros2_control Development Team
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
#include <string>
#include <vector>

#include "multi_interface_joint/multi_interface_joint.hpp"

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ::testing;  // NOLINT

class TestMultiInterfaceJoint : public Test
{
};

TEST_F(TestMultiInterfaceJoint, empty_initialized)
{
  multi_interface_joint::MultiInterfaceJoint joint;
  EXPECT_TRUE(joint.get_command_interfaces().empty());
  EXPECT_TRUE(joint.get_state_interfaces().empty());
}

TEST_F(TestMultiInterfaceJoint, wrong_initialized)
{
  {
    multi_interface_joint::MultiInterfaceJoint joint;
    hardware_interface::components::ComponentInfo joint_info;
    EXPECT_EQ(hardware_interface::return_type::INTERFACE_NOT_PROVIDED, joint.configure(joint_info));
  }

  {
    hardware_interface::components::ComponentInfo joint_info;
    joint_info.command_interfaces.push_back("position");
    joint_info.command_interfaces.push_back("position");
    multi_interface_joint::MultiInterfaceJoint joint;
    EXPECT_EQ(hardware_interface::return_type::INTERFACE_DUPLICATES, joint.configure(joint_info));
  }

  {
    hardware_interface::components::ComponentInfo joint_info;
    joint_info.state_interfaces.push_back("position");
    joint_info.state_interfaces.push_back("position");
    multi_interface_joint::MultiInterfaceJoint joint;
    EXPECT_EQ(hardware_interface::return_type::INTERFACE_DUPLICATES, joint.configure(joint_info));
  }
}

TEST_F(TestMultiInterfaceJoint, correct_initialized)
{
  {
    hardware_interface::components::ComponentInfo joint_info;
    joint_info.command_interfaces.push_back("position");
    joint_info.command_interfaces.push_back("velocity");
    joint_info.state_interfaces.push_back("effort");
    multi_interface_joint::MultiInterfaceJoint joint;
    EXPECT_EQ(hardware_interface::return_type::OK, joint.configure(joint_info));
    ASSERT_EQ(2u, joint.get_command_interfaces().size());
    ASSERT_EQ(1u, joint.get_state_interfaces().size());
    EXPECT_EQ("position", joint.get_command_interfaces()[0]);
    EXPECT_EQ("velocity", joint.get_command_interfaces()[1]);
    EXPECT_EQ("effort", joint.get_state_interfaces()[0]);
  }
}

TEST_F(TestMultiInterfaceJoint, getters_and_setters)
{
  multi_interface_joint::MultiInterfaceJoint joint;

  hardware_interface::components::ComponentInfo joint_info;
  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  ASSERT_EQ(
    hardware_interface::return_type::OK, joint.configure(joint_info));

  // Command getters and setters
  std::vector<std::string> interfaces;
  std::vector<double> input;
  input.push_back(2.1);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_PROVIDED,
    joint.set_command(input, interfaces));

  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    joint.set_command(input, interfaces));

  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    joint.set_command(input, interfaces));

  interfaces.clear();
  input.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  input.push_back(1.02);
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.set_command(input, interfaces));

  std::vector<double> output;
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.get_command(output, interfaces));
  ASSERT_EQ(1u, output.size());
  EXPECT_EQ(1.02, output[0]);

  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    joint.get_command(output, interfaces));

  input.clear();
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    joint.set_command(input));
  input.push_back(5.77);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    joint.set_command(input));

  input.clear();
  input.push_back(1.2);
  input.push_back(0.4);
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.set_command(input));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.get_command(output));
  ASSERT_EQ(2u, output.size());
  EXPECT_EQ(0.4, output[1]);

  // State getters and setters
  interfaces.clear();
  input.clear();
  input.push_back(2.1);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_PROVIDED,
    joint.set_state(input, interfaces));
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    joint.set_state(input, interfaces));
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    joint.set_state(input, interfaces));

  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  input.clear();
  input.push_back(1.2);
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.set_state(input, interfaces));

  output.clear();
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.get_state(output, interfaces));
  ASSERT_THAT(1u, output.size());
  EXPECT_EQ(1.2, output[0]);

  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    joint.get_state(output, interfaces));

  input.clear();
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    joint.set_state(input));
  input.push_back(2.1);
  input.push_back(1.02);
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.set_state(input));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    joint.get_state(output));
  ASSERT_EQ(2u, output.size());
  EXPECT_EQ(2.1, output[0]);
}
