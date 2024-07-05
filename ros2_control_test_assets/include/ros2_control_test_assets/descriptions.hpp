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

#ifndef ROS2_CONTROL_TEST_ASSETS__DESCRIPTIONS_HPP_
#define ROS2_CONTROL_TEST_ASSETS__DESCRIPTIONS_HPP_

#include <string>
#include <vector>

namespace ros2_control_test_assets
{
const auto urdf_head =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
    <safety_controller soft_lower_limit="-1.5" soft_upper_limit="0.5" k_position="10.0" k_velocity="20.0"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint3" type="revolute">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link2"/>
    <child link="link3"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link3">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
  <link name="tool_link">
  </link>
)";

const auto urdf_head_revolute_missing_limits =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
)";

const auto urdf_head_continuous_missing_limits =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="continuous">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint2" type="continuous">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
)";

const auto urdf_head_continuous_with_limits =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="continuous">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" velocity="0.2"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint2" type="continuous">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" velocity="0.2"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
)";

const auto urdf_head_prismatic_missing_limits =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="prismatic">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
)";

const auto urdf_head_mimic =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <!-- Used for fixing robot -->
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <mimic joint="joint1" multiplier="-2" offset="0"/>
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <material name="DarkGrey">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
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
  <link name="tool_link">
  </link>
)";

const auto urdf_tail =
  R"(
</robot>
)";

const auto hardware_resources =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
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
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

const auto async_hardware_resources =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator" is_async="true">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor" is_async="true">
    <hardware>
      <plugin>test_sensor</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="velocity"/>
    </sensor>
  </ros2_control>
  <ros2_control name="TestSystemHardware" type="system" is_async="true">
    <hardware>
      <plugin>test_system</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

const auto uninitializable_hardware_resources =
  R"(
  <ros2_control name="TestUninitializableActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_uninitializable_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
    </joint>
  </ros2_control>
  <ros2_control name="TestUninitializableSensorHardware" type="sensor">
    <hardware>
      <plugin>test_uninitializable_sensor</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="velocity"/>
    </sensor>
  </ros2_control>
  <ros2_control name="TestUninitializableSystemHardware" type="system">
    <hardware>
      <plugin>test_uninitializable_system</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
  </ros2_control>
)";

const auto hardware_resources_not_existing_actuator_plugin =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator23</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
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
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

const auto hardware_resources_not_existing_sensor_plugin =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <plugin>test_sensor23</plugin>
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
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";
const auto hardware_resources_not_existing_system_plugin =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
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
      <plugin>test_system23</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

const auto hardware_resources_duplicated_component =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
    </joint>
  </ros2_control>
  <ros2_control name="TestActuatorHardware" type="sensor">
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
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

const auto hardware_resources_actuator_initializaion_error =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="does_not_exist"/>
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
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

const auto hardware_resources_sensor_initializaion_error =
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
      <state_interface name="does_not_exist"/>
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
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

const auto hardware_resources_system_initializaion_error =
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
      <state_interface name="does_not_exist"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="does_not_exist"/>
    </joint>
  </ros2_control>
)";

const auto hardware_resources_missing_state_keys =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="does_not_exist"/>
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
      <state_interface name="does_not_exist"/>
      <state_interface name="does_not_exist"/>
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
      <command_interface name="max_acceleration"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="does_not_exist"/>
    </joint>
  </ros2_control>
)";

const auto hardware_resources_missing_command_keys =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="does_not_exist"/>
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
      <command_interface name="max_acceleration"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <command_interface name="does_not_exist"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

const auto diffbot_urdf =
  R"(
<?xml version="1.0" ?>
<robot name="robot">
  <!-- Space btw top of beam and the each joint -->
  <!-- Links: inertial,visual,collision -->
  <link name="base_link">
    <inertial>
      <!-- origin is relative -->
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="5"/>
      <inertia ixx="5" ixy="0" ixz="0" iyy="5" iyz="0" izz="5"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <!-- origin is relative -->
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
  <link name="base_footprint">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.00000001"/>
      </geometry>
    </collision>
  </link>
  <joint name="base_footprint_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.11"/>
    <child link="base_link"/>
    <parent link="base_footprint"/>
  </joint>
  <link name="wheel_0_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.086" radius="0.11"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.086" radius="0.11"/>
      </geometry>
    </collision>
  </link>
  <joint name="wheel_left" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_0_link"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" velocity="1.0"/>
  </joint>
  <!-- Transmission is important to link the joints and the controller -->
  <transmission name="wheel_left_trans" type="SimpleTransmission">
    <actuator name="wheel_left_motor"/>
    <joint name="wheel_left"/>
    <mechanicalReduction>1</mechanicalReduction>
    <motorTorqueConstant>1</motorTorqueConstant>
  </transmission>
  <gazebo reference="wheel_0_link">
    <material>Gazebo/Red</material>
  </gazebo>
  <link name="wheel_1_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.086" radius="0.11"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.086" radius="0.11"/>
      </geometry>
    </collision>
  </link>
  <joint name="wheel_right" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_1_link"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="-0.2 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" velocity="1.0"/>
  </joint>
  <!-- Transmission is important to link the joints and the controller -->
  <transmission name="wheel_right_trans" type="SimpleTransmission">
    <actuator name="wheel_right_motor"/>
    <joint name="wheel_right"/>
    <mechanicalReduction>1</mechanicalReduction>
    <motorTorqueConstant>1</motorTorqueConstant>
  </transmission>
  <gazebo reference="wheel_1_link">
    <material>Gazebo/Red</material>
  </gazebo>
  <!-- Colour -->
  <gazebo reference="base_link">
    <material>Gazebo/Green</material>
  </gazebo>
  <gazebo reference="base_footprint">
    <material>Gazebo/Purple</material>
  </gazebo>
  <ros2_control name="TestActuatorHardwareLeft" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="wheel_left">
      <state_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestActuatorHardwareRight" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="wheel_right">
      <state_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestIMUSensorHardware" type="sensor">
    <hardware>
      <plugin>test_hardware_components/TestIMUSensor</plugin>
    </hardware>
    <sensor name="base_imu">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>
  </ros2_control>
</robot>
)";

const auto gripper_urdf_head =
  R"(<?xml version="1.0" ?>
<robot name="gripper">
  <link name="world"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 1 1"/>
      </geometry>
      <origin xyz="0 0 0.5"/>
      <material name="violet">
        <color rgba="0.4 0.18 0.57 1.0" />
      </material>
    </visual>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_right">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_left">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
    <parent link="base"/>
    <child link="finger_right"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
  <joint name="left_finger_joint" type="prismatic">
    <mimic joint="right_finger_joint" multiplier="2" offset="1"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
    <parent link="base"/>
    <child link="finger_left"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
)";

const auto gripper_urdf_head_no_mimic =
  R"(<?xml version="1.0" ?>
<robot name="gripper">
  <link name="world"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 1 1"/>
      </geometry>
      <origin xyz="0 0 0.5"/>
      <material name="violet">
        <color rgba="0.4 0.18 0.57 1.0" />
      </material>
    </visual>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_right">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_left">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
    <parent link="base"/>
    <child link="finger_right"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
  <joint name="left_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
    <parent link="base"/>
    <child link="finger_left"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
)";

const auto gripper_urdf_head_unknown_joint =
  R"(<?xml version="1.0" ?>
<robot name="gripper">
  <link name="world"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 1 1"/>
      </geometry>
      <origin xyz="0 0 0.5"/>
      <material name="violet">
        <color rgba="0.4 0.18 0.57 1.0" />
      </material>
    </visual>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_right">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_left">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
    <parent link="base"/>
    <child link="finger_right"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
  <joint name="left_finger_joint" type="prismatic">
    <mimic joint="middle_finger_joint" multiplier="1" offset="0"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
    <parent link="base"/>
    <child link="finger_left"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
)";

const auto gripper_urdf_head_incomplete =
  R"(<?xml version="1.0" ?>
<robot name="gripper">
  <link name="world"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 1 1"/>
      </geometry>
      <origin xyz="0 0 0.5"/>
      <material name="violet">
        <color rgba="0.4 0.18 0.57 1.0" />
      </material>
    </visual>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_right">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
    <parent link="base"/>
    <child link="finger_right"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
)";

const auto gripper_urdf_head_invalid_two_root_links =
  R"(<?xml version="1.0" ?>
<robot name="gripper">
  <link name="world"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 1 1"/>
      </geometry>
      <origin xyz="0 0 0.5"/>
      <material name="violet">
        <color rgba="0.4 0.18 0.57 1.0" />
      </material>
    </visual>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_right">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="finger_left">
    <visual>
      <geometry>
        <box size="0.4 0.1 1"/>
      </geometry>
      <origin xyz="0 0.05 0.5"/>
      <material name="grey">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
    <parent link="base"/>
    <child link="finger_right"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>
)";

const auto gripper_hardware_resources_no_command_if =
  R"(
  <ros2_control name="TestGripper" type="system">
    <joint name="right_finger_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  )";

const auto gripper_hardware_resources_mimic_true_no_command_if =
  R"(
  <ros2_control name="TestGripper" type="system">
    <joint name="right_finger_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint" mimic="true">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  )";

const auto gripper_hardware_resources_mimic_true_command_if =
  R"(
  <ros2_control name="TestGripper" type="system">
    <joint name="right_finger_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint" mimic="true">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  )";

const auto gripper_hardware_resources_mimic_false_command_if =
  R"(
  <ros2_control name="TestGripper" type="system">
    <joint name="right_finger_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint" mimic="false">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  )";

const auto minimal_robot_urdf =
  std::string(urdf_head) + std::string(hardware_resources) + std::string(urdf_tail);
const auto minimal_async_robot_urdf =
  std::string(urdf_head) + std::string(async_hardware_resources) + std::string(urdf_tail);
const auto minimal_uninitializable_robot_urdf =
  std::string(urdf_head) + std::string(uninitializable_hardware_resources) + std::string(urdf_tail);

const auto minimal_robot_not_existing_actuator_plugin =
  std::string(urdf_head) + std::string(hardware_resources_not_existing_actuator_plugin) +
  std::string(urdf_tail);
const auto minimal_robot_not_existing_sensors_plugin =
  std::string(urdf_head) + std::string(hardware_resources_not_existing_sensor_plugin) +
  std::string(urdf_tail);
const auto minimal_robot_not_existing_system_plugin =
  std::string(urdf_head) + std::string(hardware_resources_not_existing_system_plugin) +
  std::string(urdf_tail);

const auto minimal_robot_duplicated_component =
  std::string(urdf_head) + std::string(hardware_resources_duplicated_component) +
  std::string(urdf_tail);

const auto minimal_robot_actuator_initialization_error =
  std::string(urdf_head) + std::string(hardware_resources_not_existing_actuator_plugin) +
  std::string(urdf_tail);
const auto minimal_robot_sensor_initialization_error =
  std::string(urdf_head) + std::string(hardware_resources_sensor_initializaion_error) +
  std::string(urdf_tail);
const auto minimal_robot_system_initialization_error =
  std::string(urdf_head) + std::string(hardware_resources_system_initializaion_error) +
  std::string(urdf_tail);

const auto minimal_robot_missing_state_keys_urdf =
  std::string(urdf_head) + std::string(hardware_resources_missing_state_keys) +
  std::string(urdf_tail);

const auto minimal_robot_missing_command_keys_urdf =
  std::string(urdf_head) + std::string(hardware_resources_missing_command_keys) +
  std::string(urdf_tail);

[[maybe_unused]] const std::string TEST_ACTUATOR_HARDWARE_NAME = "TestActuatorHardware";
[[maybe_unused]] const std::string TEST_ACTUATOR_HARDWARE_TYPE = "actuator";
[[maybe_unused]] const std::string TEST_ACTUATOR_HARDWARE_PLUGIN_NAME = "test_actuator";
[[maybe_unused]] const std::vector<std::string> TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES = {
  "joint1/position", "joint1/max_velocity"};
[[maybe_unused]] const std::vector<std::string> TEST_ACTUATOR_HARDWARE_STATE_INTERFACES = {
  "joint1/position", "joint1/velocity", "joint1/some_unlisted_interface"};

[[maybe_unused]] const std::string TEST_SENSOR_HARDWARE_NAME = "TestSensorHardware";
[[maybe_unused]] const std::string TEST_SENSOR_HARDWARE_TYPE = "sensor";
[[maybe_unused]] const std::string TEST_SENSOR_HARDWARE_PLUGIN_NAME = "test_sensor";
[[maybe_unused]] const std::vector<std::string> TEST_SENSOR_HARDWARE_COMMAND_INTERFACES = {""};
[[maybe_unused]] const std::vector<std::string> TEST_SENSOR_HARDWARE_STATE_INTERFACES = {
  "sensor1/velocity"};

[[maybe_unused]] const std::string TEST_SYSTEM_HARDWARE_NAME = "TestSystemHardware";
[[maybe_unused]] const std::string TEST_SYSTEM_HARDWARE_TYPE = "system";
[[maybe_unused]] const std::string TEST_SYSTEM_HARDWARE_PLUGIN_NAME = "test_system";
[[maybe_unused]] const std::vector<std::string> TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES = {
  "joint2/velocity", "joint2/max_acceleration", "joint3/velocity", "configuration/max_tcp_jerk"};
[[maybe_unused]] const std::vector<std::string> TEST_SYSTEM_HARDWARE_STATE_INTERFACES = {
  "joint2/position", "joint2/velocity",     "joint2/acceleration",       "joint3/position",
  "joint3/velocity", "joint3/acceleration", "configuration/max_tcp_jerk"};

}  // namespace ros2_control_test_assets

#endif  // ROS2_CONTROL_TEST_ASSETS__DESCRIPTIONS_HPP_
