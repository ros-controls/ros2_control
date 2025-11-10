// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschraenkt)
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
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/node.hpp"

class TestResourceManagerCustomInterfaces : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestResourceManagerCustomInterfaces, custom_interfaces_validation)
{
  std::string urdf =
    R"(
    <?xml version="1.0" encoding="utf-8"?>
    <robot name="robot">
      <link name="world"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="world"/>
        <child link="link1"/>
        <limit effort="0.1" velocity="0.2" lower="-3.14" upper="3.14"/>
      </joint>
      <ros2_control name="test_robot_with_custom_interfaces" type="system">
        <hardware>
          <plugin>mock_components/GenericSystem</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="command1" data_type="bool"/>
          <command_interface name="command2" data_type="int32"/>
          <command_interface name="command3" data_type="int16"/>
          <command_interface name="command4" data_type="uint16"/>
          <command_interface name="command5" data_type="uint32"/>
          <command_interface name="command6" data_type="int64"/>
          <command_interface name="command7" data_type="uint64"/>
          <state_interface name="state1" data_type="bool"/>
          <state_interface name="state2" data_type="int32"/>
          <state_interface name="state3" data_type="int16"/>
          <state_interface name="state4" data_type="uint16"/>
          <state_interface name="state5" data_type="uint32"/>
          <state_interface name="state6" data_type="int64"/>
          <state_interface name="state7" data_type="uint64"/>
        </joint>
        <sensor name="sensor1">
          <state_interface name="state1" data_type="uint64"/>
          <state_interface name="state2" data_type="int16"/>
          <state_interface name="state3" data_type="uint16"/>
          <state_interface name="state4" data_type="uint32"/>
          <state_interface name="state5" data_type="int64"/>
        </sensor>
      </ros2_control>
    </robot>
    )";

  auto node = std::make_shared<rclcpp::Node>("test_resource_manager_custom_interfaces");
  hardware_interface::ResourceManager rm(
    urdf, node->get_node_clock_interface(), node->get_node_logging_interface(),
    false /*activate_all*/);

  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command1"), "bool");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command2"), "int32");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command3"), "int16");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command4"), "uint16");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command5"), "uint32");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command6"), "int64");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command7"), "uint64");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state1"), "bool");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state2"), "int32");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state3"), "int16");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state4"), "uint16");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state5"), "uint32");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state6"), "int64");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state7"), "uint64");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state1"), "uint64");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state2"), "int16");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state3"), "uint16");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state4"), "uint32");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state5"), "int64");
}
