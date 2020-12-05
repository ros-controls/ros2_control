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

class TestResourceManager : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    urdf_head_ =
      R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
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
)";

    urdf_tail_ =
      R"(
</robot>
)";

    hardware_resources_for_test_ =
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
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
)";

    hardware_resources_for_test_missing_keys_ =
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
      <state_interface name="does_not_exist"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <command_interface name="does_not_exist"/>
      <state_interface name="position"/>
      <state_interface name="does_not_exist"/>
    </joint>
  </ros2_control>
)";
  }

  std::string urdf_head_;
  std::string hardware_resources_for_test_;
  std::string hardware_resources_for_test_missing_keys_;
  std::string urdf_tail_;
};

TEST_F(TestResourceManager, initialization_empty) {
  ASSERT_ANY_THROW(hardware_interface::ResourceManager rm(""));
}

TEST_F(TestResourceManager, initialization_with_urdf) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

TEST_F(TestResourceManager, post_initialization_with_urdf) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  hardware_interface::ResourceManager rm;
  ASSERT_NO_THROW(rm.load_urdf(urdf));
}

TEST_F(TestResourceManager, initialization_with_urdf_manual_validation) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  // we validate the results manually
  hardware_interface::ResourceManager rm(urdf, false);

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
  auto urdf = urdf_head_ + hardware_resources_for_test_missing_keys_ + urdf_tail_;
  try {
    hardware_interface::ResourceManager rm(urdf);
    FAIL();
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    SUCCEED() << e.what();
  }
}

TEST_F(TestResourceManager, initialization_with_urdf_unclaimed) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  // we validate the results manually
  hardware_interface::ResourceManager rm(urdf);

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
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  hardware_interface::ResourceManager rm(urdf);

  std::unordered_map<std::string, hardware_interface::status> status_map;

  status_map = rm.get_components_status();
  EXPECT_EQ(status_map["TestActuatorHardware"], hardware_interface::status::CONFIGURED);
  EXPECT_EQ(status_map["TestSensorHardware"], hardware_interface::status::CONFIGURED);
  EXPECT_EQ(status_map["TestSystemHardware"], hardware_interface::status::CONFIGURED);
}

TEST_F(TestResourceManager, starting_and_stopping_resources) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  hardware_interface::ResourceManager rm(urdf);

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
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  hardware_interface::ResourceManager rm(urdf);

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
};

TEST_F(TestResourceManager, post_initialization_add_components) {
  auto urdf = urdf_head_ + hardware_resources_for_test_ + urdf_tail_;
  // we validate the results manually
  hardware_interface::ResourceManager rm(urdf, false);

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
