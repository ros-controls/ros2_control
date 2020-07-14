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

#include "hardware_interface/utils/component_parser.hpp"

using namespace ::testing;  // NOLINT

class TestComponentParser : public Test
{
protected:
  void SetUp() override
  {
    urdf_xml_head_ =
      R"(
<?xml version="1.0" encoding="utf-8"?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from minimal_robot.urdf.xacro       | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="MinimalRobot">
  <!--  <xacro:include filename="$(find ros2_control_demo_robot)/description/demo_2dof/robot_2dof.urdf.xacro" />
  <xacro:include filename="$(find ros2_control_demo_robot)/description/demo_2dof/robot_2dof.gazebo.xacro" />
  <xacro:include filename="$(find ros2_control_demo_robot)/description/demo_2dof/robot_2dof.ros2_control.xacro" />-->
  <!-- Used for fixing robot -->
  <link name="world"/>
  <gazebo reference="world">
    <static>true</static>
  </gazebo>
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
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
  </gazebo>
  <gazebo reference="link1">
    <material>Gazebo/Gray</material>
  </gazebo>
  <gazebo reference="link2">
    <material>Gazebo/Gray</material>
  </gazebo>
  <gazebo>
    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
      </plugin>
  </gazebo>
)";

    urdf_xml_tail_ =
      R"(
  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="tran2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
      <hardwareInterface>VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
)";

    valid_urdf_ros2_control_joints_only_ =
      R"(
  <ros2_control name="MinimalRobot" type="robot">
    <hardware>
      <classType>ros2_control_demo_hardware/DemoRobotHardwareMinimal</classType>
      <param name="write_for_sec">2</param>
      <param name="read_for_sec">2</param>
    </hardware>
    <joint name="joint1_position_actuator">
      <classType>ros2_control_components/PositionActuator</classType>
      <joint name="joint1">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2_position_actuator">
      <classType>ros2_control_components/PositionActuator</classType>
      <joint name="joint2">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
)";

    valid_urdf_ros2_control_joints_sensors_ =
      R"(
  <ros2_control name="MinimalRobot" type="robot">
    <hardware>
      <classType>ros2_control_demo_hardware/DemoRobotHardwareMinimal</classType>
      <param name="write_for_sec">2</param>
      <param name="read_for_sec">2</param>
    </hardware>
    <joint name="joint1_position_actuator">
      <classType>ros2_control_components/PositionActuator</classType>
      <joint name="joint1">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <sensor name="joint2_position_actuator">
      <classType>ros2_control_components/PositionSensor</classType>
      <joint name="joint2">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="min_values">-5</param>
      <param name="max_values">1</param>
    </sensor>
  </ros2_control>
)";

    valid_urdf_ros2_control_joints_sensors_hardware_ =
      R"(
  <ros2_control name="MinimalRobot"  type="robot">
    <joint name="joint1_position_actuator">
      <classType>ros2_control_components/PositionActuator</classType>
      <joint name="joint1">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
      <hardware>
        <classType>ros2_control_demo_hardware/DemoActuatorHardware</classType>
        <param name="write_for_sec">1.23</param>
      </hardware>
    </joint>
    <sensor name="joint2_position_actuator">
      <classType>ros2_control_components/PositionSensor</classType>
      <joint name="joint2">
        <interfaceName>position</interfaceName>
      </joint>
      <param name="min_values">-5</param>
      <param name="max_values">1</param>
      <hardware>
        <classType>ros2_control_demo_hardware/DemoSensorHardware</classType>
        <param name="read_for_sec">3</param>
      </hardware>
    </sensor>
  </ros2_control>
)";
  }

  std::string urdf_xml_head_, urdf_xml_tail_;
  std::string valid_urdf_ros2_control_joints_only_, valid_urdf_ros2_control_joints_sensors_;
  std::string valid_urdf_ros2_control_joints_sensors_hardware_;
};

using hardware_interface::utils::parse_system_from_urdf;

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuators)
{
  std::string urdf_to_test = urdf_xml_head_ + valid_urdf_ros2_control_joints_only_ +
    urdf_xml_tail_;
  const auto system_info = parse_system_from_urdf(urdf_to_test);

  EXPECT_EQ(system_info.name, "MinimalRobot");
  EXPECT_EQ(system_info.type, "robot");
  EXPECT_EQ(system_info.hardware_class_type, "ros2_control_demo_hardware/DemoRobotHardwareMinimal");
  ASSERT_THAT(system_info.subcomponents, SizeIs(2));
  ASSERT_THAT(system_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(system_info.hardware_parameters.at("write_for_sec"), "2");

  EXPECT_EQ(system_info.subcomponents[0].name, "joint1_position_actuator");
  EXPECT_EQ(system_info.subcomponents[0].type, "joint");
  EXPECT_EQ(system_info.subcomponents[0].class_type, "ros2_control_components/PositionActuator");
  EXPECT_EQ(system_info.subcomponents[0].joint, "joint1");
  ASSERT_THAT(system_info.subcomponents[0].parameters, SizeIs(3));

  EXPECT_EQ(system_info.subcomponents[1].parameters.at("can_read"), "True");
  EXPECT_EQ(system_info.subcomponents[1].hardware_class_type, "");
  ASSERT_THAT(system_info.subcomponents[1].hardware_parameters, SizeIs(0));
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuators_sensors)
{
  std::string urdf_to_test = urdf_xml_head_ + valid_urdf_ros2_control_joints_sensors_ +
    urdf_xml_tail_;
  const auto system_info = parse_system_from_urdf(urdf_to_test);

  EXPECT_EQ(system_info.name, "MinimalRobot");
  EXPECT_EQ(system_info.type, "robot");
  EXPECT_EQ(system_info.hardware_class_type, "ros2_control_demo_hardware/DemoRobotHardwareMinimal");
  ASSERT_THAT(system_info.subcomponents, SizeIs(2));
  ASSERT_THAT(system_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(system_info.hardware_parameters.at("write_for_sec"), "2");

  EXPECT_EQ(system_info.subcomponents[0].name, "joint1_position_actuator");
  EXPECT_EQ(system_info.subcomponents[0].type, "joint");
  EXPECT_EQ(system_info.subcomponents[0].class_type, "ros2_control_components/PositionActuator");
  EXPECT_EQ(system_info.subcomponents[0].joint, "joint1");
  ASSERT_THAT(system_info.subcomponents[0].parameters, SizeIs(3));

  EXPECT_EQ(system_info.subcomponents[1].type, "sensor");
  EXPECT_EQ(system_info.subcomponents[1].class_type, "ros2_control_components/PositionSensor");
  EXPECT_EQ(system_info.subcomponents[1].parameters.at("min_values"), "-5");
  EXPECT_EQ(system_info.subcomponents[1].hardware_class_type, "");
  ASSERT_THAT(system_info.subcomponents[1].hardware_parameters, SizeIs(0));
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuators_sensors_hardware)
{
  std::string urdf_to_test = urdf_xml_head_ + valid_urdf_ros2_control_joints_sensors_hardware_ +
    urdf_xml_tail_;
  const auto system_info = parse_system_from_urdf(urdf_to_test);

  EXPECT_EQ(system_info.name, "MinimalRobot");
  EXPECT_EQ(system_info.type, "robot");
  EXPECT_EQ(system_info.hardware_class_type, "");
  ASSERT_THAT(system_info.subcomponents, SizeIs(2));
  ASSERT_THAT(system_info.hardware_parameters, SizeIs(0));

  EXPECT_EQ(system_info.subcomponents[0].name, "joint1_position_actuator");
  EXPECT_EQ(system_info.subcomponents[0].type, "joint");
  EXPECT_EQ(system_info.subcomponents[0].class_type, "ros2_control_components/PositionActuator");
  EXPECT_EQ(system_info.subcomponents[0].joint, "joint1");
  ASSERT_THAT(system_info.subcomponents[0].parameters, SizeIs(3));
  EXPECT_EQ(system_info.subcomponents[0].hardware_class_type,
    "ros2_control_demo_hardware/DemoActuatorHardware");
  ASSERT_THAT(system_info.subcomponents[0].hardware_parameters, SizeIs(1));
  EXPECT_EQ(system_info.subcomponents[0].hardware_parameters.at("write_for_sec"), "1.23");

  EXPECT_EQ(system_info.subcomponents[1].type, "sensor");
  EXPECT_EQ(system_info.subcomponents[1].class_type, "ros2_control_components/PositionSensor");
  EXPECT_EQ(system_info.subcomponents[1].parameters.at("min_values"), "-5");
  EXPECT_EQ(system_info.subcomponents[1].hardware_class_type,
    "ros2_control_demo_hardware/DemoSensorHardware");
  ASSERT_THAT(system_info.subcomponents[1].hardware_parameters, SizeIs(1));
  EXPECT_EQ(system_info.subcomponents[1].hardware_parameters.at("read_for_sec"), "2");
}

TEST_F(TestComponentParser, empty_string_throws_error)
{
  ASSERT_THROW(parse_system_from_urdf(""), std::runtime_error);
}

TEST_F(TestComponentParser, empty_urdf_throws_error)
{
  const std::string empty_urdf =
    "<?xml version=\"1.0\"?><robot name=\"robot\" xmlns=\"http://www.ros.org\"></robot>";

  ASSERT_THROW(parse_system_from_urdf(empty_urdf), std::runtime_error);
}
