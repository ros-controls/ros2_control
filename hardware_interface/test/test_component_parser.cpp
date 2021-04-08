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

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"

using namespace ::testing;  // NOLINT

class TestComponentParser : public Test
{
protected:
  void SetUp() override
  {
  }
};

using hardware_interface::parse_control_resources_from_urdf;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

TEST_F(TestComponentParser, empty_string_throws_error)
{
  ASSERT_THROW(parse_control_resources_from_urdf(""), std::runtime_error);
}

TEST_F(TestComponentParser, empty_urdf_throws_error)
{
  const std::string empty_urdf =
    "<?xml version=\"1.0\"?><robot name=\"robot\" xmlns=\"http://www.ros.org\"></robot>";

  ASSERT_THROW(parse_control_resources_from_urdf(empty_urdf), std::runtime_error);
}

TEST_F(TestComponentParser, string_robot_not_root_throws_error)
{
  const std::string broken_xml_string =
    R"(
    <?xml version=\"1.0\"?><ros2_control name=\"robot\">><robot name=\"robot\" xmlns=\"http://www.ros.org\"></robot></ros2_control>
    )";

  ASSERT_THROW(parse_control_resources_from_urdf(broken_xml_string), std::runtime_error);
}

TEST_F(TestComponentParser, invalid_child_throws_error)
{
  const std::string broken_urdf_string =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_invalid_child +
    ros2_control_test_assets::urdf_tail;

  ASSERT_THROW(parse_control_resources_from_urdf(broken_urdf_string), std::runtime_error);
}

TEST_F(TestComponentParser, missing_attribute_throws_error)
{
  const std::string broken_urdf_string =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_missing_attribute +
    ros2_control_test_assets::urdf_tail;

  ASSERT_THROW(parse_control_resources_from_urdf(broken_urdf_string), std::runtime_error);
}

TEST_F(TestComponentParser, parameter_missing_name_throws_error)
{
  const std::string broken_urdf_string =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_parameter_missing_name +
    ros2_control_test_assets::urdf_tail;

  ASSERT_THROW(parse_control_resources_from_urdf(broken_urdf_string), std::runtime_error);
}

TEST_F(TestComponentParser, component_interface_type_empty_throws_error)
{
  const std::string broken_urdf_string =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_component_interface_type_empty +
    ros2_control_test_assets::urdf_tail;

  ASSERT_THROW(parse_control_resources_from_urdf(broken_urdf_string), std::runtime_error);
}

TEST_F(TestComponentParser, parameter_empty_throws_error)
{
  const std::string broken_urdf_string =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_parameter_empty +
    ros2_control_test_assets::urdf_tail;

  ASSERT_THROW(parse_control_resources_from_urdf(broken_urdf_string), std::runtime_error);
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_one_interface)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_one_interface +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  const auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemPositionOnly");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].max, "1");
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  ASSERT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].max, "1");
  ASSERT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].name, HW_IF_POSITION);
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_multi_interface)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_multi_interface +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  const auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemMultiInterface");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/RRBotSystemMultiInterfaceHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].name, HW_IF_VELOCITY);

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  ASSERT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(1));
  ASSERT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].name, HW_IF_EFFORT);
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_robot_with_sensor)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_sensor +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  const auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemWithSensor");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/RRBotSystemWithSensorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.sensors, SizeIs(1));

  EXPECT_EQ(hardware_info.sensors[0].name, "tcp_fts_sensor");
  EXPECT_EQ(hardware_info.sensors[0].type, "sensor");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces, SizeIs(6));
  EXPECT_THAT(hardware_info.sensors[0].command_interfaces, IsEmpty());
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[0].name, "fx");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[1].name, "fy");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[2].name, "fz");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[3].name, "tx");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[4].name, "ty");
  EXPECT_THAT(hardware_info.sensors[0].state_interfaces[5].name, "tz");

  ASSERT_THAT(hardware_info.sensors[0].parameters, SizeIs(3));
  EXPECT_EQ(hardware_info.sensors[0].parameters.at("frame_id"), "kuka_tcp");
  EXPECT_EQ(hardware_info.sensors[0].parameters.at("lower_limits"), "-100");
  EXPECT_EQ(hardware_info.sensors[0].parameters.at("upper_limits"), "100");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_robot_with_external_sensor)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_external_sensor +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(2));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "RRBotSystemPositionOnlyWithExternalSensor");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotForceTorqueSensor2D");
  EXPECT_EQ(hardware_info.type, "sensor");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "0.43");

  ASSERT_THAT(hardware_info.sensors, SizeIs(1));
  EXPECT_EQ(hardware_info.sensors[0].name, "tcp_fts_sensor");
  EXPECT_EQ(hardware_info.sensors[0].type, "sensor");
  EXPECT_EQ(hardware_info.sensors[0].parameters.at("frame_id"), "kuka_tcp");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuator_modular_robot)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_actuator_modular_robot +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(2));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint1");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/PositionActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint2");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/PositionActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "3");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuator_modular_robot_with_sensors)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_actuator_modular_robot_sensors +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(4));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint1");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/VelocityActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_VELOCITY);

  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].name, "transmission1");
  EXPECT_EQ(hardware_info.transmissions[0].type, "transmission");
// EXPECT_EQ(hardware_info.transmissions[0].class_type, "transmission_interface/SimpleTansmission");
  ASSERT_THAT(hardware_info.transmissions[0].parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint_to_actuator"), "${1024/PI}");

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint2");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/VelocityActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "3");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].max, "1");

  hardware_info = control_hardware.at(2);

  EXPECT_EQ(hardware_info.name, "RRBotModularPositionSensorJoint1");
  EXPECT_EQ(hardware_info.type, "sensor");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/PositionSensorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));
  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, IsEmpty());
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);


  hardware_info = control_hardware.at(3);

  EXPECT_EQ(hardware_info.name, "RRBotModularPositionSensorJoint2");
  EXPECT_EQ(hardware_info.type, "sensor");
  EXPECT_EQ(hardware_info.hardware_class_type, "ros2_control_demo_hardware/PositionSensorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));
  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, IsEmpty());
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_multi_joints_transmission)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_multi_joints_transmission +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "RRBotModularWrist");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/ActuatorHardwareMultiDOF");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].name, "transmission1");
  EXPECT_EQ(hardware_info.transmissions[0].type, "transmission");
  // EXPECT_EQ(
  //   hardware_info.transmissions[0].class_type,
  //   "transmission_interface/SomeComplex_2x2_Transmission");
  ASSERT_THAT(hardware_info.transmissions[0].parameters, SizeIs(6));
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joints"), "{joint1, joint2}");
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("output"), "{output1, output2}");
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint1_output1"), "1.5");
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint1_output2"), "3.2");
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint2_output1"), "3.1");
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint2_output2"), "1.4");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_sensor_only)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_sensor_only +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "CameraWithIMU");
  EXPECT_EQ(hardware_info.type, "sensor");
  EXPECT_EQ(hardware_info.hardware_class_type, "ros2_control_demo_hardware/CameraWithIMUSensor");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.sensors, SizeIs(2));
  EXPECT_EQ(hardware_info.sensors[0].name, "sensor1");
  EXPECT_EQ(hardware_info.sensors[0].type, "sensor");
  ASSERT_THAT(hardware_info.sensors[0].state_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.sensors[0].state_interfaces[0].name, "roll");
  EXPECT_EQ(hardware_info.sensors[0].state_interfaces[1].name, "pitch");
  EXPECT_EQ(hardware_info.sensors[0].state_interfaces[2].name, "yaw");

  EXPECT_EQ(hardware_info.sensors[1].name, "sensor2");
  EXPECT_EQ(hardware_info.sensors[1].type, "sensor");
  ASSERT_THAT(hardware_info.sensors[1].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.sensors[1].state_interfaces[0].name, "image");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuator_only)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_actuator_only +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "ActuatorModularJoint1");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_class_type,
    "ros2_control_demo_hardware/VelocityActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.13");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].max, "1");
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_VELOCITY);


  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].name, "transmission1");
  EXPECT_EQ(hardware_info.transmissions[0].type, "transmission");
  // EXPECT_EQ(
  //   hardware_info.transmissions[0].class_type,
  //   "transmission_interface/RotationToLinerTansmission");
  ASSERT_THAT(hardware_info.transmissions[0].parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].parameters.at("joint_to_actuator"), "${1024/PI}");
}
