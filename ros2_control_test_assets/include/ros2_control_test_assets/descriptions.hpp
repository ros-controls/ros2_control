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

namespace ros2_control_test_assets
{
const auto urdf_head =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from minimal_robot.urdf.xacro       | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
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
      <command_interface name="max_acceleration" />
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
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
      <state_interface name="does_not_exist"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
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
      <command_interface name="does_not_exist"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <command_interface name="does_not_exist"/>
      <state_interface name="position"/>
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
  <joint name="wheel_0_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_0_link"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <!-- Transmission is important to link the joints and the controller -->
  <transmission name="wheel_0_joint_trans" type="SimpleTransmission">
    <actuator name="wheel_0_joint_motor"/>
    <joint name="wheel_0_joint"/>
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
  <joint name="wheel_1_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_1_link"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="-0.2 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <!-- Transmission is important to link the joints and the controller -->
  <transmission name="wheel_1_joint_trans" type="SimpleTransmission">
    <actuator name="wheel_1_joint_motor"/>
    <joint name="wheel_1_joint"/>
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
</robot>
)";

const auto minimal_robot_urdf =
  std::string(urdf_head) + std::string(hardware_resources) + std::string(urdf_tail);

const auto minimal_robot_missing_state_keys_urdf =
  std::string(urdf_head) + std::string(hardware_resources_missing_state_keys) +
  std::string(urdf_tail);

const auto minimal_robot_missing_command_keys_urdf =
  std::string(urdf_head) + std::string(hardware_resources_missing_command_keys) +
  std::string(urdf_tail);

const auto TEST_ACTUATOR_HARDWARE_NAME = "TestActuatorHardware";
const auto TEST_ACTUATOR_HARDWARE_TYPE = "actuator";
const auto TEST_ACTUATOR_HARDWARE_CLASS_TYPE = "test_actuator";
const auto TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES = {"joint1/position", "joint1/max_velocity"};
const auto TEST_ACTUATOR_HARDWARE_STATE_INTERFACES = {"joint1/position", "joint1/velocity"};

const auto TEST_SENSOR_HARDWARE_NAME = "TestSensorHardware";
const auto TEST_SENSOR_HARDWARE_TYPE = "sensor";
const auto TEST_SENSOR_HARDWARE_CLASS_TYPE = "test_sensor";
const auto TEST_SENSOR_HARDWARE_COMMAND_INTERFACES = {""};
const auto TEST_SENSOR_HARDWARE_STATE_INTERFACES = {"sensor1/velocity"};

const auto TEST_SYSTEM_HARDWARE_NAME = "TestSystemHardware";
const auto TEST_SYSTEM_HARDWARE_TYPE = "system";
const auto TEST_SYSTEM_HARDWARE_CLASS_TYPE = "test_system";
const auto TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES = {
  "joint2/velocity", "joint2/max_acceleration", "joint3/velocity", "configuration/max_tcp_jerk"};
const auto TEST_SYSTEM_HARDWARE_STATE_INTERFACES = {
  "joint2/position", "joint3/position", "configuration/max_tcp_jerk"};

}  // namespace ros2_control_test_assets

#endif  // ROS2_CONTROL_TEST_ASSETS__DESCRIPTIONS_HPP_
