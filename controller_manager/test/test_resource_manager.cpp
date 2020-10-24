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
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <plugin>test_sensor</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="velocity"/>
    </sensor>
  </ros2_control>
  <ros2_control name="TestSystemHardware" type="system">
    <hardware>
      <plugin>test_system</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
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
  ASSERT_EQ(5u, state_handle_keys.size());
  EXPECT_TRUE(rm.state_handle_exists("joint1/position"));
  EXPECT_TRUE(rm.state_handle_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_handle_exists("sensor1/velocity"));
  EXPECT_TRUE(rm.state_handle_exists("joint2/position"));
  EXPECT_TRUE(rm.state_handle_exists("joint3/position"));

  auto command_handle_keys = rm.command_handle_keys();
  ASSERT_EQ(3u, command_handle_keys.size());
  EXPECT_TRUE(rm.command_handle_exists("joint1/position"));
  EXPECT_TRUE(rm.command_handle_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_handle_exists("joint3/velocity"));
}
