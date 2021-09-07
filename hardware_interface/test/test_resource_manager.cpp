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
#include "ros2_control_test_assets/descriptions.hpp"

class TestResourceManager : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp() {}
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
  ASSERT_EQ(10u, state_interface_keys.size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("sensor1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint3/position"));

  auto command_interface_keys = rm.command_interface_keys();
  ASSERT_EQ(3u, command_interface_keys.size());
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

TEST_F(TestResourceManager, resource_status)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  std::unordered_map<std::string, rclcpp_lifecycle::State> status_map;

  status_map = rm.get_components_states();
  EXPECT_EQ(
    status_map["TestActuatorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestActuatorHardware"].label(),
    hardware_interface::lifecycle_state_names::INACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

TEST_F(TestResourceManager, starting_and_stopping_resources)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  std::unordered_map<std::string, rclcpp_lifecycle::State> status_map;

  rm.start_components();
  status_map = rm.get_components_states();
  EXPECT_EQ(
    status_map["TestActuatorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    status_map["TestActuatorHardware"].label(), hardware_interface::lifecycle_state_names::ACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].label(), hardware_interface::lifecycle_state_names::ACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].label(), hardware_interface::lifecycle_state_names::ACTIVE);

  rm.stop_components();
  status_map = rm.get_components_states();
  EXPECT_EQ(
    status_map["TestActuatorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestActuatorHardware"].label(),
    hardware_interface::lifecycle_state_names::INACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestSensorHardware"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    status_map["TestSystemHardware"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

TEST_F(TestResourceManager, resource_claiming)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  const auto key = "joint1/position";
  EXPECT_FALSE(rm.command_interface_is_claimed(key));

  {
    auto position_command_interface = rm.claim_command_interface(key);
    EXPECT_TRUE(rm.command_interface_is_claimed(key));
    {
      EXPECT_ANY_THROW(rm.claim_command_interface(key));
    }
  }
  EXPECT_FALSE(rm.command_interface_is_claimed(key));

  // command interfaces can only be claimed once
  for (const auto & key :
       {"joint1/position", "joint1/position", "joint1/position", "joint2/velocity",
        "joint3/velocity"})
  {
    {
      auto interface = rm.claim_command_interface(key);
      EXPECT_TRUE(rm.command_interface_is_claimed(key));
      {
        EXPECT_ANY_THROW(rm.claim_command_interface(key));
      }
    }
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }

  // state interfaces can be claimed multiple times
  for (const auto & key :
       {"joint1/position", "joint1/velocity", "sensor1/velocity", "joint2/position",
        "joint3/position"})
  {
    {
      auto interface = rm.claim_state_interface(key);
      {
        EXPECT_NO_THROW(rm.claim_state_interface(key));
      }
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> interfaces;
  const auto interface_names = {"joint1/position", "joint2/velocity", "joint3/velocity"};
  for (const auto & key : interface_names)
  {
    interfaces.emplace_back(rm.claim_command_interface(key));
  }
  for (const auto & key : interface_names)
  {
    EXPECT_TRUE(rm.command_interface_is_claimed(key));
  }
  interfaces.clear();
  for (const auto & key : interface_names)
  {
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

  EXPECT_EQ(1u, rm.actuator_components_size());
  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ(1u, rm.system_components_size());

  ASSERT_EQ(10u, rm.state_interface_keys().size());
  ASSERT_EQ(3u, rm.command_interface_keys().size());

  rm.import_component(std::make_unique<ExternalComponent>());
  EXPECT_EQ(2u, rm.actuator_components_size());

  ASSERT_EQ(11u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("external_joint/external_state_interface"));
  ASSERT_EQ(4u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("external_joint/external_command_interface"));

  EXPECT_NO_THROW(rm.claim_state_interface("external_joint/external_state_interface"));
  EXPECT_NO_THROW(rm.claim_command_interface("external_joint/external_command_interface"));
}

TEST_F(TestResourceManager, default_prepare_perform_switch)
{
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);
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
