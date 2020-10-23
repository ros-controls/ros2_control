// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "resource_manager.hpp"

class TestResourceManager : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    urdf_head_ =
      R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link1">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link2">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="tool_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 1"/>
    <parent link="link2"/>
    <child link="tool_link"/>
  </joint>
)";

    urdf_tail_ =
      R"(
</robot>
)";

    test_hardware_resource_system_ =
      R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <classType>test_actuator</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <classType>test_sensor</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <classType>test_sensor</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </sensor>
  </ros2_control>
  <ros2_control name="TestSystemHardware" type="system">
    <hardware>
      <classType>test_system</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
    <joint name="joint3">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
  </ros2_control>
)";
  }

  std::string urdf_head_;
  std::string test_hardware_resource_system_;
  std::string urdf_tail_;
};

TEST_F(TestResourceManager, initialization_empty) {
  controller_manager::ResourceManager rm;
  EXPECT_EQ(0u, rm.actuator_interfaces_size());
  EXPECT_EQ(0u, rm.sensor_interfaces_size());
  EXPECT_EQ(0u, rm.system_interfaces_size());
}

TEST_F(TestResourceManager, initialization_with_urdf) {
  auto urdf = urdf_head_ + test_hardware_resource_system_ + urdf_tail_;
  controller_manager::ResourceManager rm(urdf);

  EXPECT_EQ(1u, rm.actuator_interfaces_size());
  EXPECT_EQ(1u, rm.sensor_interfaces_size());
  EXPECT_EQ(1u, rm.system_interfaces_size());

  auto state_handle_keys = rm.state_handle_keys();
  // extracting a list from an unordered_map doesn't yield deterministic results
  // sort the list to make comparison clear.
  std::sort(state_handle_keys.begin(), state_handle_keys.end());
  ASSERT_EQ(2u, state_handle_keys.size());
  EXPECT_EQ("joint1/position", state_handle_keys[0]);
  EXPECT_EQ("joint1/velocity", state_handle_keys[1]);

  auto command_handle_keys = rm.command_handle_keys();
  // extracting a list from an unordered_map doesn't yield deterministic results
  // sort the list to make comparison clear.
  std::sort(command_handle_keys.begin(), command_handle_keys.end());
  ASSERT_EQ(1u, command_handle_keys.size());
  EXPECT_EQ("joint1/velocity", command_handle_keys[0]);
}
