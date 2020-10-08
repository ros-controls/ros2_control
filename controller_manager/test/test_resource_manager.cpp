// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <string>

#include "resource_manager.hpp"

using namespace ::testing;  // NOLINT

class TestResourceManager : public Test
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

    test_hardware_resource_system_ =
      R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <classType>test_actuator_hardware</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <classType>test_sensor_hardware</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <classType>test_sensor_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </sensor>
  </ros2_control>
  <ros2_control name="TestSystemHardware" type="system">
    <hardware>
      <classType>test_system_hardware</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
    <joint name="joint3">
      <classType>test_joint_component</classType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
    </joint>
  </ros2_control>
)";
  }

  std::string urdf_head_;
  std::string test_hardware_resource_system_;
  std::string urdf_tail_;
};

TEST_F(TestResourceManager, initialization_empty) {
  controller_manager::ResourceManager rm;

  EXPECT_EQ(0u, rm.joint_components_size());
  EXPECT_EQ(0u, rm.sensor_components_size());
  EXPECT_FALSE(rm.sensor_exists("sensor1"));

  EXPECT_EQ(0u, rm.actuator_interfaces_size());
  EXPECT_EQ(0u, rm.sensor_interfaces_size());
  EXPECT_EQ(0u, rm.system_interfaces_size());
}

TEST_F(TestResourceManager, initialization_with_urdf) {
  auto urdf = urdf_head_ + test_hardware_resource_system_ + urdf_tail_;
  controller_manager::ResourceManager rm(urdf);

  EXPECT_EQ(3u, rm.joint_components_size());
  auto joint_component_names = rm.joint_components_name();
  for (const auto & joint_name : {"joint1", "joint2", "joint3"}) {
    EXPECT_NE(
      joint_component_names.end(),
      std::find(joint_component_names.begin(), joint_component_names.end(), joint_name));
  }

  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_EQ("sensor1", rm.sensor_components_name()[0]);
  EXPECT_TRUE(rm.sensor_exists("sensor1")) << "sensor1 does not exist";
  EXPECT_FALSE(rm.sensor_exists("non-existing-sensor"));

  EXPECT_EQ(1u, rm.actuator_interfaces_size());
  EXPECT_EQ("TestActuatorHardware", rm.actuator_interfaces_name()[0]);
  EXPECT_EQ(1u, rm.sensor_interfaces_size());
  EXPECT_EQ("TestSensorHardware", rm.sensor_interfaces_name()[0]);
  EXPECT_EQ(1u, rm.system_interfaces_size());
  EXPECT_EQ("TestSystemHardware", rm.system_interfaces_name()[0]);
}

TEST_F(TestResourceManager, resource_claiming) {
  auto urdf = urdf_head_ + test_hardware_resource_system_ + urdf_tail_;
  controller_manager::ResourceManager rm(urdf);

  EXPECT_EQ(1u, rm.sensor_components_size());
  EXPECT_FALSE(rm.sensor_is_claimed("sensor1"));

  {
    auto sensor1 = rm.claim_sensor("sensor1");
    EXPECT_TRUE(rm.sensor_is_claimed("sensor1"));
    try {
      auto sensor1_again = rm.claim_sensor("sensor1");
      FAIL();
    } catch (const std::runtime_error &) {
      SUCCEED();
    }
  }
  EXPECT_FALSE(rm.sensor_is_claimed("sensor1"));
}
