// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Denis Stogl

#include <gmock/gmock.h>
#include <string>
#include <unordered_map>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"

class TestGenericSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_robot_2dof_ =
      R"(
  <ros2_control name="GenericRobot2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericRobot</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">0.7854</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="start_position">0.7854</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_asymetric_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="start_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="acceleration"/>
      <state_interface name="position"/>
      <param name="start_position">0.7854</param>
      <param name="start_acceleration">0.8554</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_functional_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="start_position">0.0</param>
      <param name="start_velocity">0.0</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="start_position">0.0</param>
      <param name="start_velocity">0.0</param>
    </joint>
  </ros2_control>
)";
  }

  std::string hardware_robot_2dof_;
  std::string hardware_system_2dof_;
  std::string hardware_system_2dof_asymetric_;
  std::string hardware_system_2dof_functional_;
};

TEST_F(TestGenericSystem, load_generic_system_2dof) {
  auto urdf =
    ros2_control_test_assets::urdf_xml_head_ + hardware_system_2dof_ +
    ros2_control_test_assets::urdf_xml_tail_;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

TEST_F(TestGenericSystem, load_generic_robot_2dof) {
  auto urdf =
    ros2_control_test_assets::urdf_xml_head_ + hardware_robot_2dof_ +
    ros2_control_test_assets::urdf_xml_tail_;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

// Test inspired by hardware_interface/test_resource_manager.cpp
TEST_F(TestGenericSystem, load_generic_system_2dof_check_symetric_interfaces) {
  auto urdf =
    ros2_control_test_assets::urdf_xml_head_ + hardware_system_2dof_ +
    ros2_control_test_assets::urdf_xml_tail_;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  auto state_interface_keys = rm.state_interface_keys();
  ASSERT_EQ(2u, state_interface_keys.size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));

  auto command_interface_keys = rm.state_interface_keys();
  ASSERT_EQ(2u, command_interface_keys.size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");

  ASSERT_EQ(1.57, j1p_s.get_value());
  ASSERT_EQ(0.7854, j2p_s.get_value());
  ASSERT_EQ(1.57, j1p_c.get_value());
  ASSERT_EQ(0.7854, j2p_c.get_value());
}

// Test inspired by hardware_interface/test_resource_manager.cpp
TEST_F(TestGenericSystem, load_generic_system_2dof_check_asymetric_interfaces) {
  auto urdf =
    ros2_control_test_assets::urdf_xml_head_ + hardware_system_2dof_asymetric_ +
    ros2_control_test_assets::urdf_xml_tail_;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  auto state_interface_keys = rm.state_interface_keys();
  ASSERT_EQ(2u, state_interface_keys.size());
  EXPECT_FALSE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint1/acceleration"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/acceleration"));

  auto command_interface_keys = rm.state_interface_keys();
  ASSERT_EQ(2u, command_interface_keys.size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_FALSE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm.command_interface_exists("joint1/acceleration"));
  EXPECT_FALSE(rm.command_interface_exists("joint2/position"));
  EXPECT_FALSE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/acceleration"));

  // Check initial values
  ASSERT_ANY_THROW(rm.claim_state_interface("joint1/position"));
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  ASSERT_ANY_THROW(rm.claim_state_interface("joint1/acceleration"));
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  ASSERT_ANY_THROW(rm.claim_state_interface("joint2/velocity"));
  ASSERT_ANY_THROW(rm.claim_state_interface("joint2/acceleration"));

  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  ASSERT_ANY_THROW(rm.claim_command_interface("joint1/velocity"));
  ASSERT_ANY_THROW(rm.claim_command_interface("joint1/acceleration"));
  ASSERT_ANY_THROW(rm.claim_command_interface("joint2/position"));
  ASSERT_ANY_THROW(rm.claim_command_interface("joint2/position"));
  hardware_interface::LoanedCommandInterface j2a_c = rm.claim_command_interface(
    "joint2/acceleration");

  EXPECT_TRUE(std::isnan(j1v_s.get_value()));
  ASSERT_EQ(0.7854, j2p_s.get_value());
  ASSERT_EQ(1.57, j1p_c.get_value());
  ASSERT_EQ(0.8554, j2a_c.get_value());
}

// Test inspired by hardware_interface/test_resource_manager.cpp
TEST_F(TestGenericSystem, load_generic_system_2dof_check_functionality) {
  auto urdf =
    ros2_control_test_assets::urdf_xml_head_ + hardware_system_2dof_functional_ +
    ros2_control_test_assets::urdf_xml_tail_;
  hardware_interface::ResourceManager rm(urdf);

  // check is hardware is configured
  std::unordered_map<std::string, hardware_interface::status> status_map;
  status_map = rm.get_components_status();
  EXPECT_EQ(status_map["GenericSystem2dof"], hardware_interface::status::CONFIGURED);

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j1v_c = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j2v_c = rm.claim_command_interface("joint2/velocity");

  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.0, j1p_c.get_value());
  ASSERT_EQ(0.0, j1v_c.get_value());
  ASSERT_EQ(0.0, j2p_c.get_value());
  ASSERT_EQ(0.0, j2v_c.get_value());

  // set some new values in commands
  j1p_c.set_value(0.11);
  j1v_c.set_value(0.22);
  j2p_c.set_value(0.33);
  j2v_c.set_value(0.44);

  // State values should not be changed
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // write() does not chnage values
  rm.write();
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.44, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // set some new values in commands
  j1p_c.set_value(0.55);
  j1v_c.set_value(0.66);
  j2p_c.set_value(0.77);
  j2v_c.set_value(0.88);

  // state values should not be changed
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.44, j2v_s.get_value());
  ASSERT_EQ(0.55, j1p_c.get_value());
  ASSERT_EQ(0.66, j1v_c.get_value());
  ASSERT_EQ(0.77, j2p_c.get_value());
  ASSERT_EQ(0.88, j2v_c.get_value());
}
