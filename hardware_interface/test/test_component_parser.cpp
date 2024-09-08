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
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

#ifdef _WIN32
#define M_PI 3.1415926535897932384626433832795
#endif

using namespace ::testing;  // NOLINT

class TestComponentParser : public Test
{
protected:
  void SetUp() override {}
};

using hardware_interface::HW_IF_ACCELERATION;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::parse_control_resources_from_urdf;

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
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
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

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(2));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint1", "joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
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
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
    "ros2_control_demo_hardware/RRBotSystemMultiInterfaceHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].initial_value, "1.2");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].initial_value, "3.4");
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].name, HW_IF_VELOCITY);

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  ASSERT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(1));
  ASSERT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(3));
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].name, HW_IF_EFFORT);

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(2));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint1", "joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    // effort and velocity limits won't change as they are above the main URDF hard limits
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
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
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/RRBotSystemWithSensorHardware");
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

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(2));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint1", "joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
}

TEST_F(
  TestComponentParser,
  successfully_parse_valid_urdf_system_multi_interface_custom_interface_parameters)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::
      valid_urdf_ros2_control_system_multi_interface_and_custom_interface_parameters +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  const auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemMultiInterface");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
    "ros2_control_demo_hardware/RRBotSystemMultiInterfaceHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_EQ(hardware_info.joints[0].parameters.size(), 3);
  EXPECT_EQ(hardware_info.joints[0].parameters.at("modbus_server_ip"), "1.1.1.1");
  EXPECT_EQ(hardware_info.joints[0].parameters.at("modbus_server_port"), "1234");
  EXPECT_EQ(hardware_info.joints[0].parameters.at("use_persistent_connection"), "true");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(3));
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(3));
  // CommandInterfaces of joints
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].initial_value, "1.2");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].max, "1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].parameters.size(), 5);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].parameters.at("register"), "1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].parameters.at("register_size"), "2");

  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].initial_value, "3.4");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].min, "-1.5");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].max, "1.5");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].parameters.size(), 5);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].parameters.at("register"), "2");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].parameters.at("register_size"), "4");

  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].min, "-0.5");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].max, "0.5");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].initial_value, "");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].parameters.size(), 2);

  // StateInterfaces of joints
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].parameters.size(), 2);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].parameters.at("register"), "3");
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].parameters.at("register_size"), "2");

  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].parameters.size(), 2);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].parameters.at("register"), "4");
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[1].parameters.at("register_size"), "4");

  EXPECT_EQ(hardware_info.joints[0].state_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[2].parameters.size(), 0);

  // Second Joint
  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  EXPECT_EQ(hardware_info.joints[1].parameters.size(), 2);
  EXPECT_EQ(hardware_info.joints[1].parameters.at("modbus_server_ip"), "192.168.178.123");
  EXPECT_EQ(hardware_info.joints[1].parameters.at("modbus_server_port"), "4321");
  ASSERT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(1));
  ASSERT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(3));
  // CommandInterfaces
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].initial_value, "");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].max, "1");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].parameters.size(), 4);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].parameters.at("register"), "20");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].parameters.at("data_type"), "int32_t");
  // StateInterfaces of joints
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].parameters.size(), 2);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].parameters.at("register"), "21");
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].parameters.at("data_type"), "int32_t");

  EXPECT_EQ(hardware_info.joints[1].state_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[1].parameters.size(), 0);

  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].parameters.size(), 2);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].parameters.at("register"), "21");
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[2].parameters.at("data_type"), "int32_t");
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
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
    "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "2");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(2));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint1", "joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotForceTorqueSensor2D");
  EXPECT_EQ(hardware_info.type, "sensor");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "0.43");

  ASSERT_THAT(hardware_info.sensors, SizeIs(1));
  EXPECT_EQ(hardware_info.sensors[0].name, "tcp_fts_sensor");
  EXPECT_EQ(hardware_info.sensors[0].type, "sensor");
  EXPECT_EQ(hardware_info.sensors[0].parameters.at("frame_id"), "kuka_tcp");
  ASSERT_THAT(hardware_info.limits, SizeIs(0));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
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
  EXPECT_EQ(hardware_info.group, "Hardware Group");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/PositionActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
  for (const auto & joint : {"joint1"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
  }

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint2");
  EXPECT_EQ(hardware_info.group, "Hardware Group");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/PositionActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "3");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    // position limits will be limited than original range, as their range is limited in the
    // ros2_control tags
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
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
  EXPECT_EQ(hardware_info.group, "Hardware Group 1");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/VelocityActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_VELOCITY);

  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].name, "transmission1");
  EXPECT_EQ(hardware_info.transmissions[0].type, "transmission_interface/SimpleTansmission");
  ASSERT_THAT(hardware_info.transmissions[0].joints, SizeIs(1));
  EXPECT_THAT(hardware_info.transmissions[0].joints[0].name, "joint1");
  EXPECT_THAT(
    hardware_info.transmissions[0].joints[0].mechanical_reduction, DoubleNear(1024.0 / M_PI, 0.01));
  ASSERT_THAT(hardware_info.transmissions[0].actuators, SizeIs(1));
  EXPECT_THAT(hardware_info.transmissions[0].actuators[0].name, "actuator1");
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
  for (const auto & joint : {"joint1"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(M_PI, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-M_PI, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    // effort and velocity limits won't change as they are above the main URDF hard limits
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
  }

  hardware_info = control_hardware.at(1);

  EXPECT_EQ(hardware_info.name, "RRBotModularJoint2");
  EXPECT_EQ(hardware_info.group, "Hardware Group 2");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/VelocityActuatorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "3");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].min, "-1");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].max, "1");
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(M_PI, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-M_PI, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    // effort and velocity limits won't change as they are above the main URDF hard limits
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }

  hardware_info = control_hardware.at(2);

  EXPECT_EQ(hardware_info.name, "RRBotModularPositionSensorJoint1");
  EXPECT_EQ(hardware_info.group, "Hardware Group 1");
  EXPECT_EQ(hardware_info.type, "sensor");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/PositionSensorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));
  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, IsEmpty());
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
  for (const auto & joint : {"joint1"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(M_PI, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-M_PI, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    // effort and velocity limits won't change as they are above the main URDF hard limits
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
  }

  hardware_info = control_hardware.at(3);

  EXPECT_EQ(hardware_info.name, "RRBotModularPositionSensorJoint2");
  EXPECT_EQ(hardware_info.group, "Hardware Group 2");
  EXPECT_EQ(hardware_info.type, "sensor");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/PositionSensorHardware");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(1));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  ASSERT_THAT(hardware_info.sensors, SizeIs(0));
  ASSERT_THAT(hardware_info.joints, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].name, "joint2");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  ASSERT_THAT(hardware_info.joints[0].command_interfaces, IsEmpty());
  ASSERT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  for (const auto & joint : {"joint2"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(M_PI, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-M_PI, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    // effort and velocity limits won't change as they are above the main URDF hard limits
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
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
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/ActuatorHardwareMultiDOF");
  ASSERT_THAT(hardware_info.hardware_parameters, SizeIs(2));
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.23");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));
  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  EXPECT_EQ(hardware_info.transmissions[0].name, "transmission1");
  EXPECT_EQ(hardware_info.transmissions[0].type, "transmission_interface/DifferentialTransmission");
  ASSERT_THAT(hardware_info.transmissions[0].joints, SizeIs(2));
  EXPECT_EQ(hardware_info.transmissions[0].joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.transmissions[0].joints[0].role, "joint1");
  EXPECT_EQ(hardware_info.transmissions[0].joints[0].mechanical_reduction, 10.0);
  EXPECT_EQ(hardware_info.transmissions[0].joints[0].offset, 0.5);
  EXPECT_EQ(hardware_info.transmissions[0].joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.transmissions[0].joints[1].role, "joint2");
  EXPECT_EQ(hardware_info.transmissions[0].joints[1].mechanical_reduction, 50.0);
  EXPECT_EQ(hardware_info.transmissions[0].joints[1].offset, 0.0);

  ASSERT_THAT(hardware_info.transmissions[0].actuators, SizeIs(2));
  EXPECT_EQ(hardware_info.transmissions[0].actuators[0].name, "joint1_motor");
  EXPECT_EQ(hardware_info.transmissions[0].actuators[0].role, "actuator1");
  EXPECT_EQ(hardware_info.transmissions[0].actuators[1].name, "joint2_motor");
  EXPECT_EQ(hardware_info.transmissions[0].actuators[1].role, "actuator2");
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_sensor_only)
{
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                             ros2_control_test_assets::valid_urdf_ros2_control_sensor_only +
                             ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "CameraWithIMU");
  EXPECT_EQ(hardware_info.type, "sensor");
  ASSERT_THAT(hardware_info.group, IsEmpty());
  EXPECT_EQ(hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/CameraWithIMUSensor");
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
  // There will be no limits as the ros2_control tag has only sensor info
  ASSERT_THAT(hardware_info.limits, SizeIs(0));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_actuator_only)
{
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                             ros2_control_test_assets::valid_urdf_ros2_control_actuator_only +
                             ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.name, "ActuatorModularJoint1");
  EXPECT_EQ(hardware_info.type, "actuator");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/VelocityActuatorHardware");
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
  const auto transmission = hardware_info.transmissions[0];
  EXPECT_EQ(transmission.name, "transmission1");
  EXPECT_EQ(transmission.type, "transmission_interface/RotationToLinerTansmission");
  EXPECT_THAT(transmission.joints, SizeIs(1));
  const auto joint = transmission.joints[0];
  EXPECT_EQ(joint.name, "joint1");
  EXPECT_EQ(joint.role, "joint1");
  EXPECT_THAT(joint.state_interfaces, ElementsAre("velocity"));
  EXPECT_THAT(joint.command_interfaces, ElementsAre("velocity"));
  EXPECT_THAT(joint.mechanical_reduction, DoubleEq(325.949));
  EXPECT_THAT(joint.offset, DoubleEq(0.0));
  EXPECT_THAT(transmission.actuators, SizeIs(1));
  const auto actuator = transmission.actuators[0];
  EXPECT_EQ(actuator.name, "actuator1");
  EXPECT_EQ(actuator.role, "actuator1");
  EXPECT_THAT(actuator.state_interfaces, ContainerEq(joint.state_interfaces));
  EXPECT_THAT(actuator.command_interfaces, ContainerEq(joint.command_interfaces));
  EXPECT_THAT(actuator.offset, DoubleEq(0.0));
  ASSERT_THAT(transmission.parameters, SizeIs(1));
  EXPECT_EQ(transmission.parameters.at("additional_special_parameter"), "1337");

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_position_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").max_position, DoubleNear(M_PI, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").min_position, DoubleNear(-M_PI, 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_effort_limits);
  // effort and velocity limits won't change as they are above the main URDF hard limits
  EXPECT_THAT(hardware_info.limits.at("joint1").max_velocity, DoubleNear(0.2, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_effort, DoubleNear(0.1, 1e-5));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
}

TEST_F(TestComponentParser, successfully_parse_locale_independent_double)
{
  // Set to locale with comma-separated decimals
  std::setlocale(LC_NUMERIC, "de_DE.UTF-8");

  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                             ros2_control_test_assets::valid_urdf_ros2_control_actuator_only +
                             ros2_control_test_assets::urdf_tail;

  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  const auto hardware_info = control_hardware.at(0);

  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "1.13");

  ASSERT_THAT(hardware_info.transmissions, SizeIs(1));
  const auto transmission = hardware_info.transmissions[0];
  EXPECT_THAT(transmission.joints, SizeIs(1));
  const auto joint = transmission.joints[0];

  // Test that we still parse doubles using dot notation
  EXPECT_THAT(joint.mechanical_reduction, DoubleEq(325.949));
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_robot_with_gpio)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_gpio +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemWithGPIO");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/RRBotSystemWithGPIOHardware");

  ASSERT_THAT(hardware_info.joints, SizeIs(2));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");

  ASSERT_THAT(hardware_info.gpios, SizeIs(2));

  EXPECT_EQ(hardware_info.gpios[0].name, "flange_analog_IOs");
  EXPECT_EQ(hardware_info.gpios[0].type, "gpio");
  EXPECT_THAT(hardware_info.gpios[0].state_interfaces, SizeIs(3));
  EXPECT_THAT(hardware_info.gpios[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].name, "analog_output1");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].name, "analog_input1");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[2].name, "analog_input2");

  EXPECT_EQ(hardware_info.gpios[1].name, "flange_vacuum");
  EXPECT_EQ(hardware_info.gpios[1].type, "gpio");
  EXPECT_THAT(hardware_info.gpios[1].state_interfaces, SizeIs(1));
  EXPECT_THAT(hardware_info.gpios[1].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.gpios[1].state_interfaces[0].name, "vacuum");
  EXPECT_EQ(hardware_info.gpios[1].state_interfaces[0].initial_value, "1.0");
  EXPECT_EQ(hardware_info.gpios[1].command_interfaces[0].name, "vacuum");

  EXPECT_THAT(hardware_info.transmissions, IsEmpty());

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(2));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(1));
  char joint2_name[] = "joint2";
  for (const auto & joint : {"joint1", "joint2"})
  {
    // Position limits are limited in the ros2_control tag
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(1.0, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
    if (strcmp(joint, joint2_name) == 0)
    {
      EXPECT_THAT(hardware_info.soft_limits.at(joint).max_position, DoubleNear(0.5, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).min_position, DoubleNear(-1.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_position, DoubleNear(10.0, 1e-5));
      EXPECT_THAT(hardware_info.soft_limits.at(joint).k_velocity, DoubleNear(20.0, 1e-5));
    }
  }
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_with_size_and_data_type)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_size_and_data_type +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemWithSizeAndDataType");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
    "ros2_control_demo_hardware/RRBotSystemWithSizeAndDataType");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].size, 1);
  EXPECT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].size, 1);

  ASSERT_THAT(hardware_info.gpios, SizeIs(1));

  EXPECT_EQ(hardware_info.gpios[0].name, "flange_IOS");
  EXPECT_EQ(hardware_info.gpios[0].type, "gpio");
  EXPECT_THAT(hardware_info.gpios[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].name, "digital_output");
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].data_type, "bool");
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].size, 2);
  EXPECT_THAT(hardware_info.gpios[0].state_interfaces, SizeIs(2));
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].name, "analog_input");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].size, 3);
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].name, "image");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].data_type, "cv::Mat");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].size, 1);
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_and_disabled_interfaces)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::
      valid_urdf_ros2_control_system_robot_with_gpio_and_disabled_interface_limits +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemWithGPIO");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/RRBotSystemWithGPIOHardware");

  ASSERT_THAT(hardware_info.joints, SizeIs(3));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(5));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].name, "jerk");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].size, 1);
  EXPECT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].size, 1);

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  EXPECT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(5));
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].name, "jerk");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].size, 1);
  EXPECT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].size, 1);

  EXPECT_EQ(hardware_info.joints[2].name, "joint3");
  EXPECT_EQ(hardware_info.joints[2].type, "joint");
  EXPECT_THAT(hardware_info.joints[2].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].size, 1);

  ASSERT_THAT(hardware_info.gpios, SizeIs(1));

  EXPECT_EQ(hardware_info.gpios[0].name, "flange_IOS");
  EXPECT_EQ(hardware_info.gpios[0].type, "gpio");
  EXPECT_THAT(hardware_info.gpios[0].command_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].name, "digital_output");
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].data_type, "bool");
  EXPECT_EQ(hardware_info.gpios[0].command_interfaces[0].size, 2);
  EXPECT_THAT(hardware_info.gpios[0].state_interfaces, SizeIs(2));
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].name, "analog_input");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[0].size, 3);
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].name, "image");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].data_type, "cv::Mat");
  EXPECT_EQ(hardware_info.gpios[0].state_interfaces[1].size, 1);

  EXPECT_FALSE(hardware_info.limits.at("joint1").has_position_limits);
  EXPECT_THAT(
    hardware_info.limits.at("joint1").max_position,
    DoubleNear(std::numeric_limits<double>::max(), 1e-5));
  EXPECT_THAT(
    hardware_info.limits.at("joint1").min_position,
    DoubleNear(-std::numeric_limits<double>::max(), 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_effort_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_acceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_deceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").max_velocity, DoubleNear(0.05, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_effort, DoubleNear(0.1, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_acceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_deceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_jerk, DoubleNear(5.0, 1e-5));

  EXPECT_FALSE(hardware_info.limits.at("joint2").has_position_limits);
  EXPECT_THAT(
    hardware_info.limits.at("joint2").max_position,
    DoubleNear(std::numeric_limits<double>::max(), 1e-5));
  EXPECT_THAT(
    hardware_info.limits.at("joint2").min_position,
    DoubleNear(-std::numeric_limits<double>::max(), 1e-5));
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_velocity_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_effort_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_acceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint2").max_velocity, DoubleNear(0.2, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint2").max_effort, DoubleNear(0.1, 1e-5));
  EXPECT_THAT(hardware_info.soft_limits.at("joint2").max_position, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.soft_limits.at("joint2").min_position, DoubleNear(-1.5, 1e-5));
  EXPECT_THAT(hardware_info.soft_limits.at("joint2").k_position, DoubleNear(10.0, 1e-5));
  EXPECT_THAT(hardware_info.soft_limits.at("joint2").k_velocity, DoubleNear(20.0, 1e-5));

  EXPECT_TRUE(hardware_info.limits.at("joint3").has_position_limits);
  EXPECT_THAT(hardware_info.limits.at("joint3").min_position, DoubleNear(-M_PI, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_position, DoubleNear(M_PI, 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_effort_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_acceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint3").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint3").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint3").max_velocity, DoubleNear(0.2, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_effort, DoubleNear(0.1, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_acceleration, DoubleNear(1.0, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_deceleration, DoubleNear(1.0, 1e-5));
}

TEST_F(TestComponentParser, successfully_parse_valid_urdf_system_and_unavailable_interfaces)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_unavailable_interfaces +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "RRBotSystemWithGPIO");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name, "ros2_control_demo_hardware/RRBotSystemWithGPIOHardware");

  ASSERT_THAT(hardware_info.joints, SizeIs(3));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_THAT(hardware_info.joints[0].command_interfaces, SizeIs(5));
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[1].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[2].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[3].size, 1);
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].name, "jerk");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[4].size, 1);
  EXPECT_THAT(hardware_info.joints[0].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[0].state_interfaces[0].size, 1);

  EXPECT_EQ(hardware_info.joints[1].name, "joint2");
  EXPECT_EQ(hardware_info.joints[1].type, "joint");
  EXPECT_THAT(hardware_info.joints[1].command_interfaces, SizeIs(5));
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[0].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].name, HW_IF_VELOCITY);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[1].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].name, HW_IF_EFFORT);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[2].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[3].size, 1);
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].name, "jerk");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].command_interfaces[4].size, 1);
  EXPECT_THAT(hardware_info.joints[1].state_interfaces, SizeIs(1));
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].name, HW_IF_POSITION);
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[1].state_interfaces[0].size, 1);

  EXPECT_EQ(hardware_info.joints[2].name, "joint3");
  EXPECT_EQ(hardware_info.joints[2].type, "joint");
  EXPECT_THAT(hardware_info.joints[2].command_interfaces, SizeIs(2));
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].name, HW_IF_ACCELERATION);
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].data_type, "double");
  EXPECT_EQ(hardware_info.joints[2].command_interfaces[0].size, 1);

  ASSERT_THAT(hardware_info.gpios, SizeIs(0));

  EXPECT_FALSE(hardware_info.limits.at("joint1").has_position_limits);
  EXPECT_THAT(
    hardware_info.limits.at("joint1").max_position,
    DoubleNear(std::numeric_limits<double>::max(), 1e-5));
  EXPECT_THAT(
    hardware_info.limits.at("joint1").min_position,
    DoubleNear(-std::numeric_limits<double>::max(), 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_effort_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_acceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_deceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").max_velocity, DoubleNear(0.05, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_effort, DoubleNear(0.1, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_acceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_deceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_jerk, DoubleNear(5.0, 1e-5));

  EXPECT_FALSE(hardware_info.limits.at("joint2").has_position_limits);
  EXPECT_THAT(
    hardware_info.limits.at("joint2").max_position,
    DoubleNear(std::numeric_limits<double>::max(), 1e-5));
  EXPECT_THAT(
    hardware_info.limits.at("joint2").min_position,
    DoubleNear(-std::numeric_limits<double>::max(), 1e-5));
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_velocity_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_effort_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_acceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint2").max_velocity, DoubleNear(0.2, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint2").max_effort, DoubleNear(0.1, 1e-5));

  EXPECT_TRUE(hardware_info.limits.at("joint3").has_position_limits);
  EXPECT_THAT(hardware_info.limits.at("joint3").min_position, DoubleNear(-M_PI, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_position, DoubleNear(M_PI, 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_effort_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint3").has_acceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint3").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint3").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint3").max_velocity, DoubleNear(0.2, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_effort, DoubleNear(0.1, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_acceleration, DoubleNear(1.0, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint3").max_deceleration, DoubleNear(1.0, 1e-5));
}

TEST_F(TestComponentParser, throw_on_parse_invalid_urdf_system_missing_limits)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head_revolute_missing_limits) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_one_interface +
    ros2_control_test_assets::urdf_tail;
  EXPECT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);

  urdf_to_test = std::string(ros2_control_test_assets::urdf_head_prismatic_missing_limits) +
                 ros2_control_test_assets::valid_urdf_ros2_control_system_one_interface +
                 ros2_control_test_assets::urdf_tail;
  EXPECT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, throw_on_parse_urdf_system_with_command_fixed_joint)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf_ros2_control_system_with_command_fixed_joint +
    ros2_control_test_assets::urdf_tail;
  EXPECT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, successfully_parse_urdf_system_continuous_missing_limits)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head_continuous_missing_limits) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_all_interfaces +
    ros2_control_test_assets::urdf_tail;
  EXPECT_NO_THROW(parse_control_resources_from_urdf(urdf_to_test));

  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  ASSERT_GT(hardware_info.limits.count("joint1"), 0);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_position_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").min_position, DoubleNear(-1.0, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_position, DoubleNear(1.0, 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_effort_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_acceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_deceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").max_velocity, DoubleNear(0.05, 1e-5))
    << "velocity constraint from ros2_control_tag";
  EXPECT_THAT(hardware_info.limits.at("joint1").max_effort, DoubleNear(0.2, 1e-5))
    << "effort constraint from ros2_control_tag";
  EXPECT_THAT(hardware_info.limits.at("joint1").max_acceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_deceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_jerk, DoubleNear(5.0, 1e-5));

  ASSERT_GT(hardware_info.limits.count("joint2"), 0);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_position_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_velocity_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_effort_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_acceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_jerk_limits);
}

TEST_F(TestComponentParser, successfully_parse_urdf_system_continuous_with_limits)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head_continuous_with_limits) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_all_interfaces +
    ros2_control_test_assets::urdf_tail;
  EXPECT_NO_THROW(parse_control_resources_from_urdf(urdf_to_test));

  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  ASSERT_GT(hardware_info.limits.count("joint1"), 0);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_position_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").min_position, DoubleNear(-1.0, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_position, DoubleNear(1.0, 1e-5));
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_effort_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_acceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_deceleration_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint1").has_jerk_limits);
  EXPECT_THAT(hardware_info.limits.at("joint1").max_velocity, DoubleNear(0.05, 1e-5))
    << "velocity URDF constraint overridden by ros2_control tag";
  EXPECT_THAT(hardware_info.limits.at("joint1").max_effort, DoubleNear(0.1, 1e-5))
    << "effort constraint from URDF";
  EXPECT_THAT(hardware_info.limits.at("joint1").max_acceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_deceleration, DoubleNear(0.5, 1e-5));
  EXPECT_THAT(hardware_info.limits.at("joint1").max_jerk, DoubleNear(5.0, 1e-5));

  ASSERT_GT(hardware_info.limits.count("joint2"), 0);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_position_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint2").has_velocity_limits);
  EXPECT_TRUE(hardware_info.limits.at("joint2").has_effort_limits);
  EXPECT_THAT(hardware_info.limits.at("joint2").max_velocity, DoubleNear(0.2, 1e-5))
    << "velocity constraint from URDF";
  EXPECT_THAT(hardware_info.limits.at("joint2").max_effort, DoubleNear(0.1, 1e-5))
    << "effort constraint from URDF";
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_acceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_deceleration_limits);
  EXPECT_FALSE(hardware_info.limits.at("joint2").has_jerk_limits);
}

TEST_F(TestComponentParser, successfully_parse_parameter_empty)
{
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_parameter_empty +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(control_hardware, SizeIs(1));
  auto hardware_info = control_hardware.front();

  EXPECT_EQ(hardware_info.name, "2DOF_System_Robot_Position_Only");
  EXPECT_EQ(hardware_info.type, "system");
  EXPECT_EQ(
    hardware_info.hardware_plugin_name,
    "ros2_control_demo_hardware/2DOF_System_Hardware_Position_Only");

  ASSERT_THAT(hardware_info.joints, SizeIs(1));

  EXPECT_EQ(hardware_info.joints[0].name, "joint1");
  EXPECT_EQ(hardware_info.joints[0].type, "joint");
  EXPECT_EQ(hardware_info.joints[0].command_interfaces[0].name, "position");

  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_write_for_sec"), "");
  EXPECT_EQ(hardware_info.hardware_parameters.at("example_param_read_for_sec"), "2");

  // Verify limits parsed from the URDF
  ASSERT_THAT(hardware_info.limits, SizeIs(1));
  ASSERT_THAT(hardware_info.soft_limits, SizeIs(0));
  for (const auto & joint : {"joint1"})
  {
    EXPECT_TRUE(hardware_info.limits.at(joint).has_position_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_position, DoubleNear(M_PI, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).min_position, DoubleNear(-M_PI, 1e-5));
    EXPECT_TRUE(hardware_info.limits.at(joint).has_velocity_limits);
    EXPECT_TRUE(hardware_info.limits.at(joint).has_effort_limits);
    EXPECT_THAT(hardware_info.limits.at(joint).max_velocity, DoubleNear(0.2, 1e-5));
    EXPECT_THAT(hardware_info.limits.at(joint).max_effort, DoubleNear(0.1, 1e-5));
  }
}

TEST_F(TestComponentParser, negative_size_throws_error)
{
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                             ros2_control_test_assets::invalid_urdf2_ros2_control_illegal_size +
                             ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, noninteger_size_throws_error)
{
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                             ros2_control_test_assets::invalid_urdf2_ros2_control_illegal_size2 +
                             ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, transmission_and_component_joint_mismatch_throws_error)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf2_hw_transmission_joint_mismatch +
    ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, transmission_given_too_many_joints_throws_error)
{
  std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::invalid_urdf2_transmission_given_too_many_joints +
    ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, gripper_mimic_true_valid_config)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  std::vector<hardware_interface::HardwareInfo> hw_info;
  ASSERT_NO_THROW(hw_info = parse_control_resources_from_urdf(urdf_to_test));
  ASSERT_THAT(hw_info, SizeIs(1));
  ASSERT_THAT(hw_info[0].mimic_joints, SizeIs(1));
  EXPECT_DOUBLE_EQ(hw_info[0].mimic_joints[0].multiplier, 2.0);
  EXPECT_DOUBLE_EQ(hw_info[0].mimic_joints[0].offset, 1.0);
  EXPECT_EQ(hw_info[0].mimic_joints[0].mimicked_joint_index, 0);
  EXPECT_EQ(hw_info[0].mimic_joints[0].joint_index, 1);
}

TEST_F(TestComponentParser, gripper_no_mimic_valid_config)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  std::vector<hardware_interface::HardwareInfo> hw_info;
  ASSERT_NO_THROW(hw_info = parse_control_resources_from_urdf(urdf_to_test));
  ASSERT_THAT(hw_info, SizeIs(1));
  ASSERT_THAT(hw_info[0].mimic_joints, SizeIs(1));
  EXPECT_DOUBLE_EQ(hw_info[0].mimic_joints[0].multiplier, 2.0);
  EXPECT_DOUBLE_EQ(hw_info[0].mimic_joints[0].offset, 1.0);
  EXPECT_EQ(hw_info[0].mimic_joints[0].mimicked_joint_index, 0);
  EXPECT_EQ(hw_info[0].mimic_joints[0].joint_index, 1);
}

TEST_F(TestComponentParser, gripper_mimic_with_unknown_joint_throws_error)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head_unknown_joint) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, gripper_mimic_true_without_mimic_info_throws_error)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head_no_mimic) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, gripper_mimic_true_invalid_config_throws_error)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, gripper_mimic_false_valid_config)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_false_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  std::vector<hardware_interface::HardwareInfo> hw_info;
  ASSERT_NO_THROW(hw_info = parse_control_resources_from_urdf(urdf_to_test));
  ASSERT_THAT(hw_info, SizeIs(1));
  ASSERT_THAT(hw_info[0].mimic_joints, SizeIs(0));
}

/**
 * @brief Test that the parser throws an error if the URDF contains a link with no parent.
 */
TEST_F(TestComponentParser, urdf_two_root_links_throws_error)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head_invalid_two_root_links) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

/**
 * @brief Test that the parser throws an error if a joint defined in the ros2_control tag is missing
 * in the URDF
 */
TEST_F(TestComponentParser, urdf_incomplete_throws_error)
{
  const auto urdf_to_test =
    std::string(ros2_control_test_assets::gripper_urdf_head_incomplete) +
    std::string(ros2_control_test_assets::gripper_hardware_resources_mimic_true_no_command_if) +
    std::string(ros2_control_test_assets::urdf_tail);
  ASSERT_THROW(parse_control_resources_from_urdf(urdf_to_test), std::runtime_error);
}

TEST_F(TestComponentParser, parse_joint_state_interface_descriptions_from_hardware_info)
{
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_multi_joints_transmission +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);

  const auto joint_state_descriptions =
    parse_state_interface_descriptions(control_hardware[0].joints);
  EXPECT_EQ(joint_state_descriptions[0].get_prefix_name(), "joint1");
  EXPECT_EQ(joint_state_descriptions[0].get_interface_name(), "position");
  EXPECT_EQ(joint_state_descriptions[0].get_name(), "joint1/position");

  EXPECT_EQ(joint_state_descriptions[1].get_prefix_name(), "joint2");
  EXPECT_EQ(joint_state_descriptions[1].get_interface_name(), "position");
  EXPECT_EQ(joint_state_descriptions[1].get_name(), "joint2/position");
}

TEST_F(TestComponentParser, parse_joint_command_interface_descriptions_from_hardware_info)
{
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_multi_joints_transmission +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);

  const auto joint_command_descriptions =
    parse_command_interface_descriptions(control_hardware[0].joints);
  EXPECT_EQ(joint_command_descriptions[0].get_prefix_name(), "joint1");
  EXPECT_EQ(joint_command_descriptions[0].get_interface_name(), "position");
  EXPECT_EQ(joint_command_descriptions[0].get_name(), "joint1/position");
  EXPECT_EQ(joint_command_descriptions[0].interface_info.min, "-1");
  EXPECT_EQ(joint_command_descriptions[0].interface_info.max, "1");

  EXPECT_EQ(joint_command_descriptions[1].get_prefix_name(), "joint2");
  EXPECT_EQ(joint_command_descriptions[1].get_interface_name(), "position");
  EXPECT_EQ(joint_command_descriptions[1].get_name(), "joint2/position");
  EXPECT_EQ(joint_command_descriptions[1].interface_info.min, "-1");
  EXPECT_EQ(joint_command_descriptions[1].interface_info.max, "1");
}

TEST_F(TestComponentParser, parse_sensor_state_interface_descriptions_from_hardware_info)
{
  const std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
                                   ros2_control_test_assets::valid_urdf_ros2_control_sensor_only +
                                   ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);

  const auto sensor_state_descriptions =
    parse_state_interface_descriptions(control_hardware[0].sensors);
  EXPECT_EQ(sensor_state_descriptions[0].get_prefix_name(), "sensor1");
  EXPECT_EQ(sensor_state_descriptions[0].get_interface_name(), "roll");
  EXPECT_EQ(sensor_state_descriptions[0].get_name(), "sensor1/roll");
  EXPECT_EQ(sensor_state_descriptions[1].get_prefix_name(), "sensor1");
  EXPECT_EQ(sensor_state_descriptions[1].get_interface_name(), "pitch");
  EXPECT_EQ(sensor_state_descriptions[1].get_name(), "sensor1/pitch");
  EXPECT_EQ(sensor_state_descriptions[2].get_prefix_name(), "sensor1");
  EXPECT_EQ(sensor_state_descriptions[2].get_interface_name(), "yaw");
  EXPECT_EQ(sensor_state_descriptions[2].get_name(), "sensor1/yaw");

  EXPECT_EQ(sensor_state_descriptions[3].get_prefix_name(), "sensor2");
  EXPECT_EQ(sensor_state_descriptions[3].get_interface_name(), "image");
  EXPECT_EQ(sensor_state_descriptions[3].get_name(), "sensor2/image");
}

TEST_F(TestComponentParser, parse_gpio_state_interface_descriptions_from_hardware_info)
{
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_gpio +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);

  const auto gpio_state_descriptions =
    parse_state_interface_descriptions(control_hardware[0].gpios);
  EXPECT_EQ(gpio_state_descriptions[0].get_prefix_name(), "flange_analog_IOs");
  EXPECT_EQ(gpio_state_descriptions[0].get_interface_name(), "analog_output1");
  EXPECT_EQ(gpio_state_descriptions[0].get_name(), "flange_analog_IOs/analog_output1");
  EXPECT_EQ(gpio_state_descriptions[1].get_prefix_name(), "flange_analog_IOs");
  EXPECT_EQ(gpio_state_descriptions[1].get_interface_name(), "analog_input1");
  EXPECT_EQ(gpio_state_descriptions[1].get_name(), "flange_analog_IOs/analog_input1");
  EXPECT_EQ(gpio_state_descriptions[2].get_prefix_name(), "flange_analog_IOs");
  EXPECT_EQ(gpio_state_descriptions[2].get_interface_name(), "analog_input2");
  EXPECT_EQ(gpio_state_descriptions[2].get_name(), "flange_analog_IOs/analog_input2");

  EXPECT_EQ(gpio_state_descriptions[3].get_prefix_name(), "flange_vacuum");
  EXPECT_EQ(gpio_state_descriptions[3].get_interface_name(), "vacuum");
  EXPECT_EQ(gpio_state_descriptions[3].get_name(), "flange_vacuum/vacuum");
}

TEST_F(TestComponentParser, parse_gpio_command_interface_descriptions_from_hardware_info)
{
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_system_robot_with_gpio +
    ros2_control_test_assets::urdf_tail;
  const auto control_hardware = parse_control_resources_from_urdf(urdf_to_test);

  const auto gpio_state_descriptions =
    parse_command_interface_descriptions(control_hardware[0].gpios);
  EXPECT_EQ(gpio_state_descriptions[0].get_prefix_name(), "flange_analog_IOs");
  EXPECT_EQ(gpio_state_descriptions[0].get_interface_name(), "analog_output1");
  EXPECT_EQ(gpio_state_descriptions[0].get_name(), "flange_analog_IOs/analog_output1");

  EXPECT_EQ(gpio_state_descriptions[1].get_prefix_name(), "flange_vacuum");
  EXPECT_EQ(gpio_state_descriptions[1].get_interface_name(), "vacuum");
  EXPECT_EQ(gpio_state_descriptions[1].get_name(), "flange_vacuum/vacuum");
}
