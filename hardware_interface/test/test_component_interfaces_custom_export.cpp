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

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_components.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
constexpr unsigned int TRIGGER_READ_WRITE_ERROR_CALLS = 10000;
}  // namespace

using namespace ::testing;  // NOLINT

namespace test_components
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DummyActuatorDefault : public hardware_interface::ActuatorInterface
{
  std::string get_name() const override { return "DummyActuatorDefault"; }

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.joints[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.joints[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }
};

class DummySensorDefault : public hardware_interface::SensorInterface
{
  std::string get_name() const override { return "DummySensorDefault"; }

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.sensors[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }
};

class DummySystemDefault : public hardware_interface::SystemInterface
{
  std::string get_name() const override { return "DummySystemDefault"; }

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.joints[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.joints[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }
};

}  // namespace test_components

TEST(TestComponentInterfaces, dummy_actuator_default_custom_export)
{
  hardware_interface::Actuator actuator_hw(
    std::make_unique<test_components::DummyActuatorDefault>());
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_dummy_actuator_only +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo dummy_actuator = control_resources[0];
  rclcpp::Logger logger = rclcpp::get_logger("test_actuator_component");
  auto state = actuator_hw.initialize(dummy_actuator, logger, nullptr);

  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  auto state_interfaces = actuator_hw.export_state_interfaces();
  ASSERT_EQ(3u, state_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/some_unlisted_interface");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/some_unlisted_interface", state_interfaces[position]->get_name());
    EXPECT_EQ("some_unlisted_interface", state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }

  auto command_interfaces = actuator_hw.export_command_interfaces();
  ASSERT_EQ(2u, command_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/some_unlisted_interface");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/some_unlisted_interface", command_interfaces[position]->get_name());
    EXPECT_EQ("some_unlisted_interface", command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
}

TEST(TestComponentInterfaces, dummy_sensor_default_custom_export)
{
  hardware_interface::Sensor sensor_hw(std::make_unique<test_components::DummySensorDefault>());

  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_voltage_sensor_only +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo voltage_sensor_res = control_resources[0];
  rclcpp::Logger logger = rclcpp::get_logger("test_sensor_component");
  auto state = sensor_hw.initialize(voltage_sensor_res, logger, nullptr);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  auto state_interfaces = sensor_hw.export_state_interfaces();
  ASSERT_EQ(2u, state_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/voltage");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/voltage", state_interfaces[position]->get_name());
    EXPECT_EQ("voltage", state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
    EXPECT_TRUE(std::isnan(state_interfaces[position]->get_value()));
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/some_unlisted_interface");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/some_unlisted_interface", state_interfaces[position]->get_name());
    EXPECT_EQ("some_unlisted_interface", state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
}

TEST(TestComponentInterfaces, dummy_system_default_custom_export)
{
  hardware_interface::System system_hw(std::make_unique<test_components::DummySystemDefault>());

  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_dummy_system_robot +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo dummy_system = control_resources[0];
  rclcpp::Logger logger = rclcpp::get_logger("test_system_component");
  auto state = system_hw.initialize(dummy_system, logger, nullptr);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  auto state_interfaces = system_hw.export_state_interfaces();
  ASSERT_EQ(7u, state_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint2/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint2/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint3/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint3/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/some_unlisted_interface");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/some_unlisted_interface", state_interfaces[position]->get_name());
    EXPECT_EQ("some_unlisted_interface", state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }

  auto command_interfaces = system_hw.export_command_interfaces();
  ASSERT_EQ(4u, command_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint2/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint3/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/some_unlisted_interface");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/some_unlisted_interface", command_interfaces[position]->get_name());
    EXPECT_EQ("some_unlisted_interface", command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
}
