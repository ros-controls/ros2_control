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

#include "gmock/gmock.h"

#include "multi_interface_joint/multi_interface_joint.hpp"

#include "hardware_interface/components/component_info.hpp"

class TestMultiInterfaceJoint : public testing::Test
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

//  joint_info.name = "DummyMultiJoint";
//  joint_info.parameters["max_position"] = "3.14";
//  joint_info.parameters["min_position"] = "-3.14";
//  joint_info.parameters["max_velocity"] = "1.14";
//  joint_info.parameters["min_velocity"] = "-1.14";
//
//  EXPECT_EQ(joint.configure(joint_info), return_type::ERROR);
//
//  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
//  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//
//  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
//
//  ASSERT_THAT(joint.get_command_interfaces(), SizeIs(2));
//  EXPECT_EQ(joint.get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
//  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(0));
//
//  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);
//  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
//  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(2));
//  EXPECT_EQ(joint.get_command_interfaces()[1], hardware_interface::HW_IF_VELOCITY);
//
//  // Command getters and setters
//  std::vector<std::string> interfaces;
//  std::vector<double> input;
//  input.push_back(2.1);
//  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
//  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
//  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_FOUND);
//  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//  interfaces.clear();
//  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
//  input.clear();
//  input.push_back(1.02);
//  EXPECT_EQ(joint.set_command(input, interfaces), return_type::OK);
//
//  std::vector<double> output;
//  EXPECT_EQ(joint.get_command(output, interfaces), return_type::OK);
//  ASSERT_THAT(output, SizeIs(1));
//  EXPECT_EQ(output[0], 1.02);
//  interfaces.clear();
//  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
//  EXPECT_EQ(joint.get_command(output, interfaces), return_type::INTERFACE_NOT_FOUND);
//
//  input.clear();
//  EXPECT_EQ(joint.set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//  input.push_back(5.77);
//  EXPECT_EQ(joint.set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//  input.clear();
//  input.push_back(1.2);
//  input.push_back(0.4);
//  EXPECT_EQ(joint.set_command(input), return_type::OK);
//
//  EXPECT_EQ(joint.get_command(output), return_type::OK);
//  ASSERT_THAT(output, SizeIs(2));
//  EXPECT_EQ(output[1], 0.4);
//
//  // State getters and setters
//  interfaces.clear();
//  input.clear();
//  input.push_back(2.1);
//  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
//  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
//  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_FOUND);
//  interfaces.push_back(hardware_interface::HW_IF_POSITION);
//  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//  interfaces.clear();
//  interfaces.push_back(hardware_interface::HW_IF_POSITION);
//  input.clear();
//  input.push_back(1.2);
//  EXPECT_EQ(joint.set_state(input, interfaces), return_type::OK);
//
//  output.clear();
//  EXPECT_EQ(joint.get_state(output, interfaces), return_type::OK);
//  ASSERT_THAT(output, SizeIs(1));
//  EXPECT_EQ(output[0], 1.2);
//  interfaces.clear();
//  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
//  EXPECT_EQ(joint.get_state(output, interfaces), return_type::INTERFACE_NOT_FOUND);
//
//  input.clear();
//  EXPECT_EQ(joint.set_state(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
//  input.push_back(2.1);
//  input.push_back(1.02);
//  EXPECT_EQ(joint.set_state(input), return_type::OK);
//
//  EXPECT_EQ(joint.get_state(output), return_type::OK);
//  ASSERT_THAT(output, SizeIs(2));
//  EXPECT_EQ(output[0], 2.1);
//}
