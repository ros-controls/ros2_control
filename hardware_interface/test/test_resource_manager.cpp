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

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestResourceManager : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
  }
};

TEST_F(TestResourceManager, initialization_empty) {
  ASSERT_ANY_THROW(hardware_interface::ResourceManager rm(""));
}

TEST_F(TestResourceManager, initialization_with_urdf) {
  ASSERT_NO_THROW(
    hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf));
}

TEST_F(TestResourceManager, post_initialization_with_urdf) {
  hardware_interface::ResourceManager rm;
  ASSERT_NO_THROW(rm.load_urdf(ros2_control_test_assets::minimal_robot_urdf));
}

TEST_F(TestResourceManager, initialization_with_urdf_manual_validation) {
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

TEST_F(TestResourceManager, initialization_with_wrong_urdf) {
  try {
    hardware_interface::ResourceManager rm(
      ros2_control_test_assets::minimal_robot_missing_keys_urdf);
    FAIL();
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    SUCCEED() << e.what();
  }
}

TEST_F(TestResourceManager, initialization_with_urdf_unclaimed) {
  // we validate the results manually
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  auto command_interface_keys = rm.command_interface_keys();
  for (const auto & key : command_interface_keys) {
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
  // state interfaces don't have to be locked, hence any arbitrary key
  // should return false.
  auto state_interface_keys = rm.state_interface_keys();
  for (const auto & key : state_interface_keys) {
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
}

TEST_F(TestResourceManager, resource_status) {
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  std::unordered_map<std::string, hardware_interface::status> status_map;

  status_map = rm.get_components_status();
  EXPECT_EQ(status_map["TestActuatorHardware"], hardware_interface::status::CONFIGURED);
  EXPECT_EQ(status_map["TestSensorHardware"], hardware_interface::status::CONFIGURED);
  EXPECT_EQ(status_map["TestSystemHardware"], hardware_interface::status::CONFIGURED);
}

TEST_F(TestResourceManager, starting_and_stopping_resources) {
  hardware_interface::ResourceManager rm(ros2_control_test_assets::minimal_robot_urdf);

  std::unordered_map<std::string, hardware_interface::status> status_map;

  rm.start_components();
  status_map = rm.get_components_status();
  EXPECT_EQ(status_map["TestActuatorHardware"], hardware_interface::status::STARTED);
  EXPECT_EQ(status_map["TestSensorHardware"], hardware_interface::status::STARTED);
  EXPECT_EQ(status_map["TestSystemHardware"], hardware_interface::status::STARTED);

  rm.stop_components();
  status_map = rm.get_components_status();
  EXPECT_EQ(status_map["TestActuatorHardware"], hardware_interface::status::STOPPED);
  EXPECT_EQ(status_map["TestSensorHardware"], hardware_interface::status::STOPPED);
  EXPECT_EQ(status_map["TestSystemHardware"], hardware_interface::status::STOPPED);
}

TEST_F(TestResourceManager, resource_claiming) {
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
  for (const auto & key : interface_names) {
    interfaces.emplace_back(rm.claim_command_interface(key));
  }
  for (const auto & key : interface_names) {
    EXPECT_TRUE(rm.command_interface_is_claimed(key));
  }
  interfaces.clear();
  for (const auto & key : interface_names) {
    EXPECT_FALSE(rm.command_interface_is_claimed(key));
  }
}

class ExternalComponent : public hardware_interface::ActuatorInterface
{
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo &) override
  {
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "external_joint", "external_state_interface", nullptr));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "external_joint", "external_command_interface", nullptr));

    return command_interfaces;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const override
  {
    return "ExternalComponent";
  }

  hardware_interface::status get_status() const override
  {
    return hardware_interface::status::UNKNOWN;
  }

  hardware_interface::return_type read() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type accept_command_resource_claim(const std::vector<std::string> & interfaces) override
  {
    (void)interfaces;
    return hardware_interface::return_type::OK;
  }
};

TEST_F(TestResourceManager, post_initialization_add_components) {
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
