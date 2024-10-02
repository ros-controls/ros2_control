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

// Authors: Karsten Knese, Denis Stogl

#include "test_resource_manager.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_PLUGIN_NAME;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_PLUGIN_NAME;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_PLUGIN_NAME;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_TYPE;
using testing::SizeIs;

auto configure_components =
  [](TestableResourceManager & rm, const std::vector<std::string> & components = {})
{
  return set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto activate_components =
  [](TestableResourceManager & rm, const std::vector<std::string> & components = {})
{
  return set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components =
  [](TestableResourceManager & rm, const std::vector<std::string> & components = {})
{
  return set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto cleanup_components =
  [](TestableResourceManager & rm, const std::vector<std::string> & components = {})
{
  return set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
};

auto shutdown_components =
  [](TestableResourceManager & rm, const std::vector<std::string> & components = {})
{
  return set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
};

TEST_F(ResourceManagerTest, initialization_empty)
{
  ASSERT_ANY_THROW(TestableResourceManager rm(node_, ""););
}

TEST_F(ResourceManagerTest, initialization_with_urdf)
{
  ASSERT_NO_THROW(TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf););
}

TEST_F(ResourceManagerTest, post_initialization_with_urdf)
{
  TestableResourceManager rm(node_);
  ASSERT_NO_THROW(rm.load_and_initialize_components(ros2_control_test_assets::minimal_robot_urdf));
}

void test_load_and_initialized_components_failure(const std::string & urdf)
{
  rclcpp::Node node = rclcpp::Node("TestableResourceManager");
  TestableResourceManager rm(node);
  ASSERT_FALSE(rm.load_and_initialize_components(urdf));

  ASSERT_FALSE(rm.are_components_initialized());

  // resource manager should also not have any components
  EXPECT_EQ(rm.actuator_components_size(), 0);
  EXPECT_EQ(rm.sensor_components_size(), 0);
  EXPECT_EQ(rm.system_components_size(), 0);

  // test actuator
  EXPECT_FALSE(rm.state_interface_exists("joint1/position"));
  EXPECT_FALSE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm.command_interface_exists("joint1/position"));
  EXPECT_FALSE(rm.command_interface_exists("joint1/max_velocity"));

  // test sensor
  EXPECT_FALSE(rm.state_interface_exists("sensor1/velocity"));

  // test system
  EXPECT_FALSE(rm.state_interface_exists("joint2/position"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/acceleration"));
  EXPECT_FALSE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_FALSE(rm.command_interface_exists("joint2/max_acceleration"));
  EXPECT_FALSE(rm.state_interface_exists("joint3/position"));
  EXPECT_FALSE(rm.state_interface_exists("joint3/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint3/acceleration"));
  EXPECT_FALSE(rm.command_interface_exists("joint3/velocity"));
  EXPECT_FALSE(rm.command_interface_exists("joint3/max_acceleration"));
}

TEST_F(ResourceManagerTest, test_uninitializable_hardware)
{
  SCOPED_TRACE("test_uninitializable_hardware_no_validation");
  // If the the hardware can not be initialized and load_and_initialize_components didn't try to
  // validate the interfaces, the interface should not show up
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_uninitializable_robot_urdf);
}

TEST_F(ResourceManagerTest, initialization_with_urdf_and_manual_validation)
{
  // we validate the results manually
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf, false);

  EXPECT_EQ(1u, rm.actuator_components_size());
  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ(1u, rm.system_components_size());

  auto state_interface_keys = rm.state_interface_keys();
  ASSERT_THAT(state_interface_keys, SizeIs(11));
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("sensor1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint3/position"));

  auto command_interface_keys = rm.command_interface_keys();
  ASSERT_THAT(command_interface_keys, SizeIs(6));
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint3/velocity"));
}

TEST_F(ResourceManagerTest, expect_validation_failure_if_not_all_interfaces_are_exported)
{
  SCOPED_TRACE("missing state keys");
  // missing state keys
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_missing_state_keys_urdf);

  SCOPED_TRACE("missing command keys");
  // missing command keys
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_missing_command_keys_urdf);
}

TEST_F(ResourceManagerTest, initialization_with_urdf_unclaimed)
{
  // we validate the results manually
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  auto command_interface_keys = rm.command_interface_keys();
  for (const auto & key : command_interface_keys)
  {
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
  // state interfaces don't have to be locked, hence any arbitrary key
  // should return false.
  auto state_interface_keys = rm.state_interface_keys();
  for (const auto & key : state_interface_keys)
  {
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
}

TEST_F(ResourceManagerTest, no_load_and_initialize_components_function_called)
{
  TestableResourceManager rm(node_);
  ASSERT_FALSE(rm.are_components_initialized());
}

TEST_F(
  ResourceManagerTest, expect_load_and_initialize_to_fail_when_a_hw_component_plugin_does_not_exist)
{
  SCOPED_TRACE("Actuator plugin does not exist");
  // Actuator
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_not_existing_actuator_plugin);

  SCOPED_TRACE("Sensor plugin does not exist");
  // Sensor
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_not_existing_sensors_plugin);

  SCOPED_TRACE("System plugin does not exist");
  // System
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_not_existing_system_plugin);
}

TEST_F(ResourceManagerTest, expect_load_and_initialize_to_fail_when_there_are_dupplicate_of_hw_comp)
{
  SCOPED_TRACE("Duplicated components");
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_duplicated_component);
}

TEST_F(
  ResourceManagerTest, expect_load_and_initialize_to_fail_when_a_hw_component_initialization_fails)
{
  SCOPED_TRACE("Actuator initialization fails");
  // Actuator
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_actuator_initialization_error);

  SCOPED_TRACE("Sensor initialization fails");
  // Sensor
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_sensor_initialization_error);

  SCOPED_TRACE("System initialization fails");
  // System
  test_load_and_initialized_components_failure(
    ros2_control_test_assets::minimal_robot_system_initialization_error);
}

TEST_F(ResourceManagerTest, load_and_initialize_components_called_if_urdf_is_valid)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);
  ASSERT_TRUE(rm.are_components_initialized());
}

TEST_F(ResourceManagerTest, load_and_initialize_components_called_if_async_urdf_is_valid)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_async_robot_urdf);
  ASSERT_TRUE(rm.are_components_initialized());
}

TEST_F(ResourceManagerTest, can_load_and_initialize_components_later)
{
  TestableResourceManager rm(node_);
  ASSERT_FALSE(rm.are_components_initialized());
  rm.load_and_initialize_components(ros2_control_test_assets::minimal_robot_urdf);
  ASSERT_TRUE(rm.are_components_initialized());
}

TEST_F(ResourceManagerTest, resource_claiming)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);
  // Activate components to get all interfaces available
  activate_components(rm);

  {
    const auto key = "joint1/position";
    EXPECT_TRUE(rm.command_interface_is_available(key));
    EXPECT_FALSE(rm.command_interface_is_claimed(key));

    {
      auto position_command_interface = rm.claim_command_interface(key);
      EXPECT_TRUE(rm.command_interface_is_available(key));
      EXPECT_TRUE(rm.command_interface_is_claimed(key));
      {
        EXPECT_ANY_THROW(rm.claim_command_interface(key));
        EXPECT_TRUE(rm.command_interface_is_available(key));
      }
    }
    EXPECT_TRUE(rm.command_interface_is_available(key));
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }

  // command interfaces can only be claimed once
  for (const auto & key :
       {"joint1/position", "joint1/position", "joint1/position", "joint2/velocity",
        "joint3/velocity"})
  {
    {
      auto interface = rm.claim_command_interface(key);
      EXPECT_TRUE(rm.command_interface_is_available(key));
      EXPECT_TRUE(rm.command_interface_is_claimed(key));
      {
        EXPECT_ANY_THROW(rm.claim_command_interface(key));
        EXPECT_TRUE(rm.command_interface_is_available(key));
      }
    }
    EXPECT_TRUE(rm.command_interface_is_available(key));
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }

  // TODO(destogl): This claim test is not true.... can not be...
  // state interfaces can be claimed multiple times
  for (const auto & key :
       {"joint1/position", "joint1/velocity", "sensor1/velocity", "joint2/position",
        "joint3/position"})
  {
    {
      EXPECT_TRUE(rm.state_interface_is_available(key));
      auto interface = rm.claim_state_interface(key);
      {
        EXPECT_TRUE(rm.state_interface_is_available(key));
        EXPECT_NO_THROW(rm.claim_state_interface(key));
      }
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> interfaces;
  const auto interface_names = {"joint1/position", "joint2/velocity", "joint3/velocity"};
  for (const auto & key : interface_names)
  {
    EXPECT_TRUE(rm.command_interface_is_available(key));
    interfaces.emplace_back(rm.claim_command_interface(key));
  }
  for (const auto & key : interface_names)
  {
    EXPECT_TRUE(rm.command_interface_is_available(key));
    EXPECT_TRUE(rm.command_interface_is_claimed(key));
  }
  interfaces.clear();
  for (const auto & key : interface_names)
  {
    EXPECT_TRUE(rm.command_interface_is_available(key));
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
}

class ExternalComponent : public hardware_interface::ActuatorInterface
{
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("external_joint", "external_state_interface", nullptr));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "external_joint", "external_command_interface", nullptr));

    return command_interfaces;
  }

  std::string get_name() const override { return "ExternalComponent"; }

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

TEST_F(ResourceManagerTest, post_initialization_add_components)
{
  // we validate the results manually
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);
  // Activate components to get all interfaces available
  activate_components(rm);

  EXPECT_EQ(1u, rm.actuator_components_size());
  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ(1u, rm.system_components_size());

  ASSERT_THAT(rm.state_interface_keys(), SizeIs(11));
  ASSERT_THAT(rm.command_interface_keys(), SizeIs(6));

  hardware_interface::HardwareInfo external_component_hw_info;
  external_component_hw_info.name = "ExternalComponent";
  external_component_hw_info.type = "actuator";
  external_component_hw_info.is_async = false;
  rm.import_component(std::make_unique<ExternalComponent>(), external_component_hw_info);
  EXPECT_EQ(2u, rm.actuator_components_size());

  ASSERT_THAT(rm.state_interface_keys(), SizeIs(12));
  EXPECT_TRUE(rm.state_interface_exists("external_joint/external_state_interface"));
  ASSERT_THAT(rm.command_interface_keys(), SizeIs(7));
  EXPECT_TRUE(rm.command_interface_exists("external_joint/external_command_interface"));

  auto status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["ExternalComponent"].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  configure_components(rm, {"ExternalComponent"});
  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["ExternalComponent"].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  activate_components(rm, {"ExternalComponent"});
  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["ExternalComponent"].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  EXPECT_NO_THROW(rm.claim_state_interface("external_joint/external_state_interface"));
  EXPECT_NO_THROW(rm.claim_command_interface("external_joint/external_command_interface"));
}

TEST_F(ResourceManagerTest, default_prepare_perform_switch)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);
  // Activate components to get all interfaces available
  activate_components(rm);

  // Default behavior for empty key lists, e.g., when a Broadcaster is activated
  EXPECT_TRUE(rm.prepare_command_mode_switch({}, {}));
  EXPECT_TRUE(rm.perform_command_mode_switch({}, {}));
}

TEST_F(ResourceManagerTest, resource_status)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  auto status_map = rm.get_components_status();

  // name
  EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].name, TEST_ACTUATOR_HARDWARE_NAME);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].name, TEST_SENSOR_HARDWARE_NAME);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].name, TEST_SYSTEM_HARDWARE_NAME);
  // type
  EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].type, TEST_ACTUATOR_HARDWARE_TYPE);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].type, TEST_SENSOR_HARDWARE_TYPE);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].type, TEST_SYSTEM_HARDWARE_TYPE);
  // plugin_name
  EXPECT_EQ(
    status_map[TEST_ACTUATOR_HARDWARE_NAME].plugin_name, TEST_ACTUATOR_HARDWARE_PLUGIN_NAME);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].plugin_name, TEST_SENSOR_HARDWARE_PLUGIN_NAME);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].plugin_name, TEST_SYSTEM_HARDWARE_PLUGIN_NAME);
  // state
  EXPECT_EQ(
    status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(
    status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
  EXPECT_EQ(
    status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(
    status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
  EXPECT_EQ(
    status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(
    status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  auto check_interfaces = [](
                            const std::vector<std::string> & registered_interfaces,
                            const std::vector<std::string> & interface_names)
  {
    for (const std::string & interface : interface_names)
    {
      auto it = std::find(registered_interfaces.begin(), registered_interfaces.end(), interface);
      EXPECT_NE(it, registered_interfaces.end());
    }
  };

  check_interfaces(
    status_map[TEST_ACTUATOR_HARDWARE_NAME].command_interfaces,
    TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES);
  EXPECT_TRUE(status_map[TEST_SENSOR_HARDWARE_NAME].command_interfaces.empty());
  check_interfaces(
    status_map[TEST_SYSTEM_HARDWARE_NAME].command_interfaces,
    TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES);

  check_interfaces(
    status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces,
    TEST_ACTUATOR_HARDWARE_STATE_INTERFACES);
  EXPECT_NE(
    std::find(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces.begin(),
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces.end(),
      "joint1/some_unlisted_interface"),
    status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces.end());
  check_interfaces(
    status_map[TEST_SENSOR_HARDWARE_NAME].state_interfaces, TEST_SENSOR_HARDWARE_STATE_INTERFACES);
  check_interfaces(
    status_map[TEST_SYSTEM_HARDWARE_NAME].state_interfaces, TEST_SYSTEM_HARDWARE_STATE_INTERFACES);
}

TEST_F(ResourceManagerTest, lifecycle_all_resources)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  // All resources start as UNCONFIGURED
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  ASSERT_THAT(configure_components(rm), ::testing::Each(hardware_interface::return_type::OK));
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  ASSERT_THAT(activate_components(rm), ::testing::Each(hardware_interface::return_type::OK));
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  ASSERT_THAT(deactivate_components(rm), ::testing::Each(hardware_interface::return_type::OK));
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  ASSERT_THAT(cleanup_components(rm), ::testing::Each(hardware_interface::return_type::OK));
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  ASSERT_THAT(shutdown_components(rm), ::testing::Each(hardware_interface::return_type::OK));
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
  }
}

TEST_F(ResourceManagerTest, lifecycle_individual_resources)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  // All resources start as UNCONFIGURED
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  configure_components(rm, {TEST_ACTUATOR_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  activate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  configure_components(rm, {TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  activate_components(rm, {TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  deactivate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  cleanup_components(rm, {TEST_SENSOR_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::INACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  shutdown_components(rm, {TEST_ACTUATOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
  }

  shutdown_components(rm, {TEST_SENSOR_HARDWARE_NAME});
  {
    auto status_map = rm.get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(),
      hardware_interface::lifecycle_state_names::FINALIZED);
  }
}

TEST_F(ResourceManagerTest, resource_availability_and_claiming_in_lifecycle)
{
  using std::placeholders::_1;
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  auto check_interfaces =
    [](const std::vector<std::string> & interface_names, auto check_method, bool expected_result)
  {
    for (const auto & interface : interface_names)
    {
      EXPECT_EQ(check_method(interface), expected_result);
    }
  };

  auto check_interface_claiming = [&](
                                    const std::vector<std::string> & state_interface_names,
                                    const std::vector<std::string> & command_interface_names,
                                    bool expected_result)
  {
    std::vector<hardware_interface::LoanedStateInterface> states;
    std::vector<hardware_interface::LoanedCommandInterface> commands;

    if (expected_result)
    {
      for (const auto & key : state_interface_names)
      {
        EXPECT_NO_THROW(states.emplace_back(rm.claim_state_interface(key)));
      }
      for (const auto & key : command_interface_names)
      {
        EXPECT_NO_THROW(commands.emplace_back(rm.claim_command_interface(key)));
      }
    }
    else
    {
      for (const auto & key : state_interface_names)
      {
        EXPECT_ANY_THROW(states.emplace_back(rm.claim_state_interface(key)));
      }
      for (const auto & key : command_interface_names)
      {
        EXPECT_ANY_THROW(commands.emplace_back(rm.claim_command_interface(key)));
      }
    }

    check_interfaces(
      command_interface_names,
      std::bind(&TestableResourceManager::command_interface_is_claimed, &rm, _1), expected_result);
  };

  // All resources start as UNCONFIGURED - All interfaces are imported but not
  // available
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
  }

  // Nothing can be claimed
  {
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES, false);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, false);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, false);
  }

  // When actuator is configured all interfaces become available
  configure_components(rm, {TEST_ACTUATOR_HARDWARE_NAME});
  {
    check_interfaces(
      {"joint1/position"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
  }

  // Can claim Actuator's interfaces
  {
    check_interface_claiming({}, {"joint1/position"}, true);
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, {"joint1/max_velocity"}, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, false);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, false);
  }

  // When actuator is activated all state- and command- interfaces become
  // available
  activate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME});
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
  }

  // Can claim all Actuator's state interfaces and command interfaces
  {
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, false);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, false);
  }

  // Check if all interfaces still exits
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
  }

  // When Sensor and System are configured their state-
  // and command- interfaces are available
  configure_components(rm, {TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      {"joint2/velocity", "joint3/velocity"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      {"joint2/max_acceleration", "configuration/max_tcp_jerk"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
  }

  // Can claim:
  // - all Actuator's state interfaces and command interfaces
  // - sensor's state interfaces
  // - system's state and command interfaces
  {
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, true);
    check_interface_claiming({}, {"joint2/velocity", "joint3/velocity"}, true);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      {"joint2/max_acceleration", "configuration/max_tcp_jerk"}, true);
  }

  // All active - everything available
  activate_components(rm, {TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
  }

  // Can claim everything
  // - actuator's state interfaces and command interfaces
  // - sensor's state interfaces
  // - system's state and non-moving command interfaces
  {
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, true);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, true);
  }

  // When deactivated - movement interfaces are not available anymore
  deactivate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_NAME});
  {
    check_interfaces(
      {"joint1/position"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
  }

  // Can claim everything
  // - actuator's state and command interfaces
  // - sensor's state interfaces
  // - system's state and command interfaces
  {
    check_interface_claiming({}, {"joint1/position"}, true);
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, {"joint1/max_velocity"}, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, true);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, true);
  }

  // When sensor is cleaned up the interfaces are not available anymore
  cleanup_components(rm, {TEST_SENSOR_HARDWARE_NAME});
  {
    check_interfaces(
      {"joint1/position"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_is_available, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_is_available, &rm, _1), true);
  }

  // Can claim everything
  // - actuator's state and command interfaces
  // - no sensor's interface
  // - system's state and command interfaces
  {
    check_interface_claiming({}, {"joint1/position"}, true);
    check_interface_claiming(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, {"joint1/max_velocity"}, true);
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, {}, false);
    check_interface_claiming(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES, true);
  }

  // Check if all interfaces still exits
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&TestableResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&TestableResourceManager::state_interface_exists, &rm, _1), true);
  }
}

TEST_F(ResourceManagerTest, managing_controllers_reference_interfaces)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf);

  std::string CONTROLLER_NAME = "test_controller";
  std::vector<std::string> REFERENCE_INTERFACE_NAMES = {"input1", "input2", "input3"};
  std::vector<std::string> FULL_REFERENCE_INTERFACE_NAMES = {
    CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[0],
    CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[1],
    CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[2]};

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;
  std::vector<double> reference_interface_values = {1.0, 2.0, 3.0};

  for (size_t i = 0; i < REFERENCE_INTERFACE_NAMES.size(); ++i)
  {
    reference_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      CONTROLLER_NAME, REFERENCE_INTERFACE_NAMES[i], &(reference_interface_values[i])));
  }

  rm.import_controller_reference_interfaces(CONTROLLER_NAME, reference_interfaces);

  ASSERT_THAT(
    rm.get_controller_reference_interface_names(CONTROLLER_NAME),
    testing::ElementsAreArray(FULL_REFERENCE_INTERFACE_NAMES));

  // check that all interfaces are imported properly
  for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
  {
    EXPECT_TRUE(rm.command_interface_exists(interface));
    EXPECT_FALSE(rm.command_interface_is_available(interface));
    EXPECT_FALSE(rm.command_interface_is_claimed(interface));
  }

  // make interface available
  rm.make_controller_reference_interfaces_available(CONTROLLER_NAME);
  for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
  {
    EXPECT_TRUE(rm.command_interface_exists(interface));
    EXPECT_TRUE(rm.command_interface_is_available(interface));
    EXPECT_FALSE(rm.command_interface_is_claimed(interface));
  }

  // try to make interfaces available from unknown controller
  EXPECT_THROW(
    rm.make_controller_reference_interfaces_available("unknown_controller"), std::out_of_range);

  // claim interfaces in a scope that deletes them after
  {
    auto claimed_itf1 = rm.claim_command_interface(FULL_REFERENCE_INTERFACE_NAMES[0]);
    auto claimed_itf3 = rm.claim_command_interface(FULL_REFERENCE_INTERFACE_NAMES[2]);

    for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
    {
      EXPECT_TRUE(rm.command_interface_exists(interface));
      EXPECT_TRUE(rm.command_interface_is_available(interface));
    }
    EXPECT_TRUE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[0]));
    EXPECT_FALSE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[1]));
    EXPECT_TRUE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[2]));

    // access interface value
    EXPECT_EQ(claimed_itf1.get_value(), 1.0);
    EXPECT_EQ(claimed_itf3.get_value(), 3.0);

    claimed_itf1.set_value(11.1);
    claimed_itf3.set_value(33.3);
    EXPECT_EQ(claimed_itf1.get_value(), 11.1);
    EXPECT_EQ(claimed_itf3.get_value(), 33.3);

    EXPECT_EQ(reference_interface_values[0], 11.1);
    EXPECT_EQ(reference_interface_values[1], 2.0);
    EXPECT_EQ(reference_interface_values[2], 33.3);
  }

  // interfaces should be released now, but still managed by resource manager
  for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
  {
    EXPECT_TRUE(rm.command_interface_exists(interface));
    EXPECT_TRUE(rm.command_interface_is_available(interface));
    EXPECT_FALSE(rm.command_interface_is_claimed(interface));
  }

  // make interfaces unavailable
  rm.make_controller_reference_interfaces_unavailable(CONTROLLER_NAME);
  for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
  {
    EXPECT_TRUE(rm.command_interface_exists(interface));
    EXPECT_FALSE(rm.command_interface_is_available(interface));
    EXPECT_FALSE(rm.command_interface_is_claimed(interface));
  }

  // try to make interfaces unavailable from unknown controller
  EXPECT_THROW(
    rm.make_controller_reference_interfaces_unavailable("unknown_controller"), std::out_of_range);

  // Last written values should stay
  EXPECT_EQ(reference_interface_values[0], 11.1);
  EXPECT_EQ(reference_interface_values[1], 2.0);
  EXPECT_EQ(reference_interface_values[2], 33.3);

  // remove reference interfaces from resource manager
  rm.remove_controller_reference_interfaces(CONTROLLER_NAME);

  // they should not exist in resource manager
  for (const auto & interface : FULL_REFERENCE_INTERFACE_NAMES)
  {
    EXPECT_FALSE(rm.command_interface_exists(interface));
    EXPECT_FALSE(rm.command_interface_is_available(interface));
  }

  // try to remove interfaces from unknown controller
  EXPECT_THROW(
    rm.make_controller_reference_interfaces_unavailable("unknown_controller"), std::out_of_range);
}

class ResourceManagerTestReadWriteError : public ResourceManagerTest
{
public:
  void setup_resource_manager_and_do_initial_checks()
  {
    rm = std::make_shared<TestableResourceManager>(
      node_, ros2_control_test_assets::minimal_robot_urdf, false);
    activate_components(*rm);

    auto status_map = rm->get_components_status();
    EXPECT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(
      status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    claimed_itfs.push_back(
      rm->claim_command_interface(TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES[0]));
    claimed_itfs.push_back(rm->claim_command_interface(TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES[0]));

    check_if_interface_available(true, true);
    // with default values read and write should run without any problems
    {
      auto [ok, failed_hardware_names] = rm->read(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
    }
    {
      auto [ok, failed_hardware_names] = rm->write(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
    }
    check_if_interface_available(true, true);
  }

  // check if all interfaces are available
  void check_if_interface_available(const bool actuator_interfaces, const bool system_interfaces)
  {
    for (const auto & interface : TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES)
    {
      EXPECT_EQ(rm->command_interface_is_available(interface), actuator_interfaces);
    }
    for (const auto & interface : TEST_ACTUATOR_HARDWARE_STATE_INTERFACES)
    {
      EXPECT_EQ(rm->state_interface_is_available(interface), actuator_interfaces);
    }
    for (const auto & interface : TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES)
    {
      EXPECT_EQ(rm->command_interface_is_available(interface), system_interfaces);
    }
    for (const auto & interface : TEST_SYSTEM_HARDWARE_STATE_INTERFACES)
    {
      EXPECT_EQ(rm->state_interface_is_available(interface), system_interfaces);
    }
  };

  using FunctionT =
    std::function<hardware_interface::HardwareReadWriteStatus(rclcpp::Time, rclcpp::Duration)>;

  void check_read_or_write_failure(
    FunctionT method_that_fails, FunctionT other_method, const double fail_value)
  {
    // define state to set components to
    rclcpp_lifecycle::State state_active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);

    // read failure for TEST_ACTUATOR_HARDWARE_NAME
    claimed_itfs[0].set_value(fail_value);
    claimed_itfs[1].set_value(fail_value - 10.0);
    {
      auto [ok, failed_hardware_names] = method_that_fails(time, duration);
      EXPECT_FALSE(ok);
      EXPECT_FALSE(failed_hardware_names.empty());
      ASSERT_THAT(
        failed_hardware_names,
        testing::ElementsAreArray(std::vector<std::string>({TEST_ACTUATOR_HARDWARE_NAME})));
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(false, true);
      rm->set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }

    // read failure for TEST_SYSTEM_HARDWARE_NAME
    claimed_itfs[0].set_value(fail_value - 10.0);
    claimed_itfs[1].set_value(fail_value);
    {
      auto [ok, failed_hardware_names] = method_that_fails(time, duration);
      EXPECT_FALSE(ok);
      EXPECT_FALSE(failed_hardware_names.empty());
      ASSERT_THAT(
        failed_hardware_names,
        testing::ElementsAreArray(std::vector<std::string>({TEST_SYSTEM_HARDWARE_NAME})));
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      check_if_interface_available(true, false);
      rm->set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }

    // read failure for both, TEST_ACTUATOR_HARDWARE_NAME and
    // TEST_SYSTEM_HARDWARE_NAME
    claimed_itfs[0].set_value(fail_value);
    claimed_itfs[1].set_value(fail_value);
    {
      auto [ok, failed_hardware_names] = method_that_fails(time, duration);
      EXPECT_FALSE(ok);
      EXPECT_FALSE(failed_hardware_names.empty());
      ASSERT_THAT(
        failed_hardware_names, testing::ElementsAreArray(std::vector<std::string>(
                                 {TEST_ACTUATOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME})));
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      check_if_interface_available(false, false);
      rm->set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active);
      rm->set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }
  }

  void check_read_or_write_deactivate(
    FunctionT method_that_deactivates, FunctionT other_method, const double deactivate_value)
  {
    // define state to set components to
    rclcpp_lifecycle::State state_active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);

    // deactivate for TEST_ACTUATOR_HARDWARE_NAME
    claimed_itfs[0].set_value(deactivate_value);
    claimed_itfs[1].set_value(deactivate_value - 10.0);
    {
      // deactivate on error
      auto [ok, failed_hardware_names] = method_that_deactivates(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);

      // reactivate
      rm->set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }

    // deactivate for TEST_SYSTEM_HARDWARE_NAME
    claimed_itfs[0].set_value(deactivate_value - 10.0);
    claimed_itfs[1].set_value(deactivate_value);
    {
      // deactivate on flag
      auto [ok, failed_hardware_names] = method_that_deactivates(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      check_if_interface_available(true, true);
      // re-activate
      rm->set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }

    // deactivate both, TEST_ACTUATOR_HARDWARE_NAME and
    // TEST_SYSTEM_HARDWARE_NAME
    claimed_itfs[0].set_value(deactivate_value);
    claimed_itfs[1].set_value(deactivate_value);
    {
      // deactivate on flag
      auto [ok, failed_hardware_names] = method_that_deactivates(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      auto status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      check_if_interface_available(true, true);
      // re-activate
      rm->set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active);
      rm->set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active);
      status_map = rm->get_components_status();
      EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      check_if_interface_available(true, true);
    }
    // write is sill OK
    {
      auto [ok, failed_hardware_names] = other_method(time, duration);
      EXPECT_TRUE(ok);
      EXPECT_TRUE(failed_hardware_names.empty());
      check_if_interface_available(true, true);
    }
  }

public:
  std::shared_ptr<TestableResourceManager> rm;
  std::vector<hardware_interface::LoanedCommandInterface> claimed_itfs;

  const rclcpp::Time time = rclcpp::Time(0);
  const rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.01);

  // values to set to hardware to simulate failure on read and write
};

TEST_F(ResourceManagerTestReadWriteError, handle_error_on_hardware_read)
{
  setup_resource_manager_and_do_initial_checks();

  using namespace std::placeholders;
  // check read methods failures
  check_read_or_write_failure(
    std::bind(&TestableResourceManager::read, rm, _1, _2),
    std::bind(&TestableResourceManager::write, rm, _1, _2), test_constants::READ_FAIL_VALUE);
}

TEST_F(ResourceManagerTestReadWriteError, handle_error_on_hardware_write)
{
  setup_resource_manager_and_do_initial_checks();

  using namespace std::placeholders;
  // check write methods failures
  check_read_or_write_failure(
    std::bind(&TestableResourceManager::write, rm, _1, _2),
    std::bind(&TestableResourceManager::read, rm, _1, _2), test_constants::WRITE_FAIL_VALUE);
}

TEST_F(ResourceManagerTestReadWriteError, handle_deactivate_on_hardware_read)
{
  setup_resource_manager_and_do_initial_checks();

  using namespace std::placeholders;
  // check read methods failures
  check_read_or_write_deactivate(
    std::bind(&TestableResourceManager::read, rm, _1, _2),
    std::bind(&TestableResourceManager::write, rm, _1, _2), test_constants::READ_DEACTIVATE_VALUE);
}

TEST_F(ResourceManagerTestReadWriteError, handle_deactivate_on_hardware_write)
{
  setup_resource_manager_and_do_initial_checks();

  using namespace std::placeholders;
  // check write methods failures
  check_read_or_write_deactivate(
    std::bind(&TestableResourceManager::write, rm, _1, _2),
    std::bind(&TestableResourceManager::read, rm, _1, _2), test_constants::WRITE_DEACTIVATE_VALUE);
}

TEST_F(ResourceManagerTest, test_caching_of_controllers_to_hardware)
{
  TestableResourceManager rm(node_, ros2_control_test_assets::minimal_robot_urdf, false);
  activate_components(rm);

  static const std::string TEST_CONTROLLER_ACTUATOR_NAME = "test_controller_actuator";
  static const std::string TEST_CONTROLLER_SYSTEM_NAME = "test_controller_system";
  static const std::string TEST_BROADCASTER_ALL_NAME = "test_broadcaster_all";
  static const std::string TEST_BROADCASTER_SENSOR_NAME = "test_broadcaster_sensor";

  rm.cache_controller_to_hardware(
    TEST_CONTROLLER_ACTUATOR_NAME, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES);
  rm.cache_controller_to_hardware(
    TEST_BROADCASTER_ALL_NAME, TEST_ACTUATOR_HARDWARE_STATE_INTERFACES);

  rm.cache_controller_to_hardware(
    TEST_CONTROLLER_SYSTEM_NAME, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES);
  rm.cache_controller_to_hardware(TEST_BROADCASTER_ALL_NAME, TEST_SYSTEM_HARDWARE_STATE_INTERFACES);

  rm.cache_controller_to_hardware(
    TEST_BROADCASTER_SENSOR_NAME, TEST_SENSOR_HARDWARE_STATE_INTERFACES);
  rm.cache_controller_to_hardware(TEST_BROADCASTER_ALL_NAME, TEST_SENSOR_HARDWARE_STATE_INTERFACES);

  {
    auto controllers = rm.get_cached_controllers_to_hardware(TEST_ACTUATOR_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::ElementsAreArray(std::vector<std::string>(
                     {TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME})));
  }

  {
    auto controllers = rm.get_cached_controllers_to_hardware(TEST_SYSTEM_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::ElementsAreArray(std::vector<std::string>(
                     {TEST_CONTROLLER_SYSTEM_NAME, TEST_BROADCASTER_ALL_NAME})));
  }

  {
    auto controllers = rm.get_cached_controllers_to_hardware(TEST_SENSOR_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::ElementsAreArray(std::vector<std::string>(
                     {TEST_BROADCASTER_SENSOR_NAME, TEST_BROADCASTER_ALL_NAME})));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
