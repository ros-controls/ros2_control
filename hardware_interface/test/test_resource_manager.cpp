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

#include <gmock/gmock.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_TYPE;

class TestResourceManager : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp() {}
};

void set_components_state(
  hardware_interface::ResourceManager & rm, const std::vector<std::string> & components,
  const uint8_t state_id, const std::string & state_name)
{
  auto int_components = components;
  if (int_components.empty())
  {
    int_components = {"TestActuatorHardware", "TestSensorHardware", "TestSystemHardware"};
  }
  for (const auto & component : int_components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components =
  [](hardware_interface::ResourceManager & rm, const std::vector<std::string> & components = {}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

auto activate_components =
  [](hardware_interface::ResourceManager & rm, const std::vector<std::string> & components = {}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  };

auto deactivate_components =
  [](hardware_interface::ResourceManager & rm, const std::vector<std::string> & components = {}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

auto cleanup_components =
  [](hardware_interface::ResourceManager & rm, const std::vector<std::string> & components = {}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  };

auto shutdown_components =
  [](hardware_interface::ResourceManager & rm, const std::vector<std::string> & components = {}) {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
      hardware_interface::lifecycle_state_names::FINALIZED);
  };

TEST_F(TestResourceManager, initialization_empty)
{
  ASSERT_ANY_THROW(hardware_interface::ResourceManager rm(""));
}

TEST_F(TestResourceManager, initialization_with_urdf)
{
  ASSERT_NO_THROW(
    hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf));
}

TEST_F(TestResourceManager, post_initialization_with_urdf)
{
  hardware_interface::ResourceManager rm;
  ASSERT_NO_THROW(rm.load_urdf(ros2_control_test_assets::minimal_robot_urdf));
}

TEST_F(TestResourceManager, initialization_with_urdf_manual_validation)
{
  // we validate the results manually
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf, false);

  EXPECT_EQ(1u, rm.actuator_components_size());
  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ(1u, rm.system_components_size());

  auto state_interface_keys = rm.state_interface_keys();
  ASSERT_EQ(11u, state_interface_keys.size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("sensor1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint3/position"));

  auto command_interface_keys = rm.command_interface_keys();
  ASSERT_EQ(6u, command_interface_keys.size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint3/velocity"));
}

TEST_F(TestResourceManager, initialization_with_wrong_urdf)
{
  // missing state keys
  {
    EXPECT_THROW(
      hardware_interface::ResourceManager rm(
        ros2_control_test_assets::minimal_robot_missing_state_keys_urdf),
      std::exception);
  }
  // missing command keys
  {
    EXPECT_THROW(
      hardware_interface::ResourceManager rm(
        ros2_control_test_assets::minimal_robot_missing_command_keys_urdf),
      std::exception);
  }
}

TEST_F(TestResourceManager, initialization_with_urdf_unclaimed)
{
  // we validate the results manually
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

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

TEST_F(TestResourceManager, resource_claiming)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);
  // Activate components to get all interfaces available
  activate_components(rm);

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

  hardware_interface::return_type read() override { return hardware_interface::return_type::OK; }

  hardware_interface::return_type write() override { return hardware_interface::return_type::OK; }
};

TEST_F(TestResourceManager, post_initialization_add_components)
{
  // we validate the results manually
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf, false);
  // Activate components to get all interfaces available
  activate_components(rm);

  EXPECT_EQ(1u, rm.actuator_components_size());
  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ(1u, rm.system_components_size());

  ASSERT_EQ(11u, rm.state_interface_keys().size());
  ASSERT_EQ(6u, rm.command_interface_keys().size());

  hardware_interface::HardwareInfo external_component_hw_info;
  external_component_hw_info.name = "ExternalComponent";
  external_component_hw_info.type = "actuator";
  rm.import_component(std::make_unique<ExternalComponent>(), external_component_hw_info);
  EXPECT_EQ(2u, rm.actuator_components_size());

  ASSERT_EQ(12u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("external_joint/external_state_interface"));
  ASSERT_EQ(7u, rm.command_interface_keys().size());
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

TEST_F(TestResourceManager, default_prepare_perform_switch)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);
  // Activate components to get all interfaces available
  activate_components(rm);

  EXPECT_TRUE(rm.prepare_command_mode_switch({""}, {""}));
  EXPECT_TRUE(rm.perform_command_mode_switch({""}, {""}));
}

const auto hardware_resources_command_modes =
  R"(
  <ros2_control name="TestSystemCommandModes" type="system">
    <hardware>
      <plugin>test_hardware_components/TestSystemCommandModes</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
const auto command_mode_urdf = std::string(ros2_control_test_assets::urdf_head) +
                               std::string(hardware_resources_command_modes) +
                               std::string(ros2_control_test_assets::urdf_tail);

TEST_F(TestResourceManager, custom_prepare_perform_switch)
{
  hardware_interface::ResourceManager rm(command_mode_urdf);
  // Scenarios defined by example criteria
  std::vector<std::string> empty_keys = {};
  std::vector<std::string> irrelevant_keys = {"elbow_joint/position", "should_joint/position"};
  std::vector<std::string> illegal_single_key = {"joint1/position"};
  std::vector<std::string> legal_keys_position = {"joint1/position", "joint2/position"};
  std::vector<std::string> legal_keys_velocity = {"joint1/velocity", "joint2/velocity"};
  // Default behavior for empty key lists
  EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, empty_keys));

  // Default behavior when given irrelevant keys
  EXPECT_TRUE(rm.prepare_command_mode_switch(irrelevant_keys, irrelevant_keys));
  EXPECT_TRUE(rm.prepare_command_mode_switch(irrelevant_keys, empty_keys));
  EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, irrelevant_keys));

  // The test hardware interface has a criteria that both joints must change mode
  EXPECT_FALSE(rm.prepare_command_mode_switch(illegal_single_key, illegal_single_key));
  EXPECT_FALSE(rm.prepare_command_mode_switch(illegal_single_key, empty_keys));
  EXPECT_FALSE(rm.prepare_command_mode_switch(empty_keys, illegal_single_key));

  // Test legal start keys
  EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_position, legal_keys_position));
  EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_velocity, legal_keys_velocity));
  EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_position, empty_keys));
  EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, legal_keys_position));
  EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_velocity, empty_keys));
  EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, legal_keys_velocity));

  // Test rejection from perform_command_mode_switch, test hardware rejects empty start sets
  EXPECT_TRUE(rm.perform_command_mode_switch(legal_keys_position, legal_keys_position));
  EXPECT_FALSE(rm.perform_command_mode_switch(empty_keys, empty_keys));
  EXPECT_FALSE(rm.perform_command_mode_switch(empty_keys, legal_keys_position));
}

TEST_F(TestResourceManager, resource_status)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  auto status_map = rm.get_components_status();

  // name
  EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].name, TEST_ACTUATOR_HARDWARE_NAME);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].name, TEST_SENSOR_HARDWARE_NAME);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].name, TEST_SYSTEM_HARDWARE_NAME);
  // type
  EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].type, TEST_ACTUATOR_HARDWARE_TYPE);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].type, TEST_SENSOR_HARDWARE_TYPE);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].type, TEST_SYSTEM_HARDWARE_TYPE);
  // class_type
  EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].class_type, TEST_ACTUATOR_HARDWARE_CLASS_TYPE);
  EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].class_type, TEST_SENSOR_HARDWARE_CLASS_TYPE);
  EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].class_type, TEST_SYSTEM_HARDWARE_CLASS_TYPE);
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
                            const std::vector<const char *> & interface_names) {
    for (const auto & interface : interface_names)
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

TEST_F(TestResourceManager, lifecycle_all_resources)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

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

  configure_components(rm);
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

  activate_components(rm);
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

  deactivate_components(rm);
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

  cleanup_components(rm);
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

  shutdown_components(rm);
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

TEST_F(TestResourceManager, lifecycle_individual_resources)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

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

  // TODO(destogl): Watch-out this fails in output, why is this not caught?!!!
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

TEST_F(TestResourceManager, resource_availability_and_claiming_in_lifecycle)
{
  using std::placeholders::_1;
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  auto check_interfaces =
    [](const std::vector<const char *> & interface_names, auto check_method, bool expected_result) {
      for (const auto & interface : interface_names)
      {
        EXPECT_EQ(check_method(interface), expected_result);
      }
    };

  auto check_interface_claiming = [&](
                                    const std::vector<const char *> & state_interface_names,
                                    const std::vector<const char *> & command_interface_names,
                                    bool expected_result) {
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
      std::bind(&hardware_interface::ResourceManager::command_interface_is_claimed, &rm, _1),
      expected_result);
  };

  // All resources start as UNCONFIGURED - All interfaces are imported but not available
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
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

  // When actuator is activated all state- and command- interfaces become available
  activate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME});
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      false);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
  }

  // When Sensor and System are configured their state-
  // and command- interfaces are available
  configure_components(rm, {TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME});
  {
    check_interfaces(
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      {"joint2/velocity", "joint3/velocity"},
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      {"joint2/max_acceleration", "configuration/max_tcp_jerk"},
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      {"joint1/max_velocity"},
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_is_available, &rm, _1),
      true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1),
      false);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_is_available, &rm, _1), true);
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
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::command_interface_exists, &rm, _1), true);

    check_interfaces(
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SENSOR_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
    check_interfaces(
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
      std::bind(&hardware_interface::ResourceManager::state_interface_exists, &rm, _1), true);
  }
}
