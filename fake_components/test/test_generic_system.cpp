// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Denis Stogl

#include <gmock/gmock.h>
#include <string>

#include "hardware_interface/resource_manager.hpp"

class TestGenericSystem : public ::testing::Test
{
protected:
  void SetUp() override
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

    hardware_system_2dof_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">0.7854</param>
    </joint>
  </ros2_control>
)";

    hardware_robot_2dof_ =
      R"(
  <ros2_control name="GenericRobot2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericRobot</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }

  std::string urdf_head_;
  std::string hardware_system_2dof_;
  std::string hardware_robot_2dof_;
  std::string urdf_tail_;
};

TEST_F(TestGenericSystem, load_generic_system_2dof) {
  auto urdf = urdf_head_ + hardware_system_2dof_ + urdf_tail_;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

TEST_F(TestGenericSystem, load_generic_robot_2dof) {
  auto urdf = urdf_head_ + hardware_system_2dof_ + urdf_tail_;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

// TEST_F(TestGenericSystem, load_generic_system_2dof_check_interfaces) {
//   auto urdf = urdf_head_ + hardware_system_2dof_ + urdf_tail_;
//   ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
//
//   EXPECT_EQ(1u, rm.sensor_components_size());
// }
