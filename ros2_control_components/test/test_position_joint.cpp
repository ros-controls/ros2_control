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
  position_joint_info.parameters["min"] = "-3.14";
  position_joint_info.parameters["max"] = "3.14";

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::OK);
  ASSERT_THAT(joint->get_command_interfaces(), SizeIs(1));
  EXPECT_EQ(joint->get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
  ASSERT_THAT(joint->get_state_interfaces(), SizeIs(1));
  EXPECT_EQ(joint->get_state_interfaces()[0], hardware_interface::HW_IF_POSITION);

  // Command setters - other commands are inherited from components::Joint
  std::vector<std::string> interfaces;
  std::vector<double> input;

  // good value
  input.push_back(2.1);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint->set_command(input), return_type::OK);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::OK);

  // above max value
  input[0] = 3.5;
  EXPECT_EQ(joint->set_command(input), return_type::VALUE_OUT_OF_LIMITS);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::VALUE_OUT_OF_LIMITS);

  // under min value
  input[0] = -3.5;
  EXPECT_EQ(joint->set_command(input), return_type::VALUE_OUT_OF_LIMITS);
  EXPECT_EQ(joint->set_command(input, interfaces), return_type::VALUE_OUT_OF_LIMITS);
}

TEST(PositionJointTest, position_joint_only_command_inteface_test)
{
  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";
  position_joint_info.parameters["min"] = "-3.14";
  position_joint_info.parameters["max"] = "3.14";
  position_joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::OK);
  ASSERT_THAT(joint->get_command_interfaces(), SizeIs(1));
  EXPECT_EQ(joint->get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
  ASSERT_THAT(joint->get_state_interfaces(), SizeIs(0));
}

TEST(PositionJointTest, position_joint_only_state_error_test)
{
  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";
  position_joint_info.parameters["min"] = "-3.14";
  position_joint_info.parameters["max"] = "3.14";
  position_joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_ONLY_STATE_DEFINED);
}

TEST(PositionJointTest, position_joint_missing_params_test)
{
  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_MISSING_PARAMETER);

  position_joint_info.parameters["min"] = "-3.14";
  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_MISSING_PARAMETER);

  position_joint_info.parameters["max"] = "3.14";
  EXPECT_EQ(joint->configure(position_joint_info), return_type::OK);
}

TEST(PositionJointTest, position_joint_too_many_interfaces_test)
{
  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

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

TEST(PositionJointTest, position_joint_wrong_interface_test)
{
  ComponentInfo position_joint_info;
  position_joint_info.name = "joint1";
  position_joint_info.parameters["min"] = "-3.14";
  position_joint_info.parameters["max"] = "3.14";
  position_joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  position_joint_info.state_interfaces.push_back(hardware_interface::HW_IF_EFFORT);

  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader(
    "hardware_interface", "hardware_interface::components::Joint");

  std::shared_ptr<hardware_interface::components::Joint> joint =
    joint_component_loader.createSharedInstance("ros2_control_components/PositionJoint");

  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_WRONG_INTERFACE);

  position_joint_info.command_interfaces.clear();
  position_joint_info.command_interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  position_joint_info.state_interfaces.clear();
  EXPECT_EQ(joint->configure(position_joint_info), return_type::COMPONENT_WRONG_INTERFACE);
}
