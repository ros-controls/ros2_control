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

#include <cmath>
#include <string>
#include <unordered_map>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestGenericSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_2dof_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
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
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="acceleration"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
      <param name="initial_acceleration">0.8554</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_standard_interfaces_ =
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
      <param name="initial_position">3.45</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">2.78</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_with_other_interface_ =
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
      <param name="initial_position">1.55</param>
      <param name="initial_velocity">0.1</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">0.65</param>
      <param name="initial_velocity">0.2</param>
    </joint>
    <joint name="voltage_output">
      <command_interface name="voltage"/>
      <state_interface name="voltage"/>
      <param name="initial_voltage">0.5</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_with_sensor_ =
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
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <sensor name="tcp_force_sensor">
      <state_interface name="fx"/>
      <state_interface name="fy"/>
      <state_interface name="tx"/>
      <state_interface name="ty"/>
      <param name="frame_id">kuka_tcp</param>
    </sensor>
  </ros2_control>
)";

    hardware_system_2dof_with_sensor_fake_command_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="fake_sensor_commands">true</param>
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
    <sensor name="tcp_force_sensor">
      <state_interface name="fx"/>
      <state_interface name="fy"/>
      <state_interface name="tx"/>
      <state_interface name="ty"/>
      <param name="frame_id">kuka_tcp</param>
    </sensor>
  </ros2_control>
)";

    hardware_system_2dof_with_sensor_fake_command_True_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="fake_sensor_commands">True</param>
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
    <sensor name="tcp_force_sensor">
      <state_interface name="fx"/>
      <state_interface name="fy"/>
      <state_interface name="tx"/>
      <state_interface name="ty"/>
      <param name="frame_id">kuka_tcp</param>
    </sensor>
  </ros2_control>
)";

    hardware_system_2dof_with_mimic_joint_ =
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
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <param name="mimic">joint1</param>
      <param name="multiplier">-2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_standard_interfaces_with_offset_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="position_state_following_offset">-3</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">3.45</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">2.78</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_missing_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="position_state_following_offset">-3</param>
      <param name="custom_interface_with_following_offset">actual_position</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">3.45</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">2.78</param>
    </joint>
  </ros2_control>
)";

    hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="position_state_following_offset">-3</param>
      <param name="custom_interface_with_following_offset">actual_position</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="actual_position"/>
      <param name="initial_position">3.45</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="actual_position"/>
      <param name="initial_position">2.78</param>
    </joint>
  </ros2_control>
)";

    valid_urdf_ros2_control_system_robot_with_gpio_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">3.45</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">2.78</param>
    </joint>
    <gpio name="flange_analog_IOs">
      <command_interface name="analog_output1" data_type="double"/>
      <state_interface name="analog_output1"/>
      <state_interface name="analog_input1"/>
      <state_interface name="analog_input2"/>
    </gpio>
    <gpio name="flange_vacuum">
      <command_interface name="vacuum"/>
      <state_interface name="vacuum" data_type="double"/>
    </gpio>
  </ros2_control>
)";

    valid_urdf_ros2_control_system_robot_with_gpio_fake_command_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>fake_components/GenericSystem</plugin>
      <param name="fake_gpio_commands">True</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">3.45</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="initial_position">2.78</param>
    </joint>
    <gpio name="flange_analog_IOs">
      <command_interface name="analog_output1" data_type="double"/>
      <state_interface name="analog_output1"/>
      <state_interface name="analog_input1"/>
      <state_interface name="analog_input2"/>
    </gpio>
    <gpio name="flange_vacuum">
      <command_interface name="vacuum"/>
      <state_interface name="vacuum" data_type="double"/>
    </gpio>
  </ros2_control>
)";
  }

  std::string hardware_robot_2dof_;
  std::string hardware_system_2dof_;
  std::string hardware_system_2dof_asymetric_;
  std::string hardware_system_2dof_standard_interfaces_;
  std::string hardware_system_2dof_with_other_interface_;
  std::string hardware_system_2dof_with_sensor_;
  std::string hardware_system_2dof_with_sensor_fake_command_;
  std::string hardware_system_2dof_with_sensor_fake_command_True_;
  std::string hardware_system_2dof_with_mimic_joint_;
  std::string hardware_system_2dof_standard_interfaces_with_offset_;
  std::string hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_;
  std::string hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_missing_;
  std::string valid_urdf_ros2_control_system_robot_with_gpio_;
  std::string valid_urdf_ros2_control_system_robot_with_gpio_fake_command_;
};

TEST_F(TestGenericSystem, load_generic_system_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

// Test inspired by hardware_interface/test_resource_manager.cpp
TEST_F(TestGenericSystem, generic_system_2dof_symetric_interfaces)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(2u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));

  ASSERT_EQ(2u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");

  ASSERT_EQ(1.57, j1p_s.get_value());
  ASSERT_EQ(0.7854, j2p_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));
}

// Test inspired by hardware_interface/test_resource_manager.cpp
TEST_F(TestGenericSystem, generic_system_2dof_asymetric_interfaces)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_asymetric_ +
              ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(2u, rm.state_interface_keys().size());
  EXPECT_FALSE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint1/acceleration"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_FALSE(rm.state_interface_exists("joint2/acceleration"));

  ASSERT_EQ(2u, rm.command_interface_keys().size());
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
  hardware_interface::LoanedCommandInterface j2a_c =
    rm.claim_command_interface("joint2/acceleration");

  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.7854, j2p_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2a_c.get_value()));
}

void generic_system_functional_test(const std::string & urdf, const double offset = 0)
{
  hardware_interface::ResourceManager rm(urdf);

  // check is hardware is configured
  std::unordered_map<std::string, rclcpp_lifecycle::State> states_map;
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j1v_c = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j2v_c = rm.claim_command_interface("joint2/velocity");

  // State interfaces without initial value are set to 0
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j1v_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2v_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j1v_c.set_value(0.22);
  j2p_c.set_value(0.33);
  j2v_c.set_value(0.44);

  // State values should not be changed
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // read() mirrors commands + offset to states
  rm.read();
  ASSERT_EQ(0.11 + offset, j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33 + offset, j2p_s.get_value());
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
  ASSERT_EQ(0.11 + offset, j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33 + offset, j2p_s.get_value());
  ASSERT_EQ(0.44, j2v_s.get_value());
  ASSERT_EQ(0.55, j1p_c.get_value());
  ASSERT_EQ(0.66, j1v_c.get_value());
  ASSERT_EQ(0.77, j2p_c.get_value());
  ASSERT_EQ(0.88, j2v_c.get_value());

  rm.start_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::ACTIVE);

  rm.stop_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

TEST_F(TestGenericSystem, generic_system_2dof_functionality)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_standard_interfaces_ +
              ros2_control_test_assets::urdf_tail;

  generic_system_functional_test(urdf);
}

TEST_F(TestGenericSystem, generic_system_2dof_other_interfaces)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_with_other_interface_ +
              ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(5u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("voltage_output/voltage"));

  ASSERT_EQ(5u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("voltage_output/voltage"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedStateInterface vo_s =
    rm.claim_state_interface("voltage_output/voltage");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface vo_c =
    rm.claim_command_interface("voltage_output/voltage");

  ASSERT_EQ(1.55, j1p_s.get_value());
  ASSERT_EQ(0.1, j1v_s.get_value());
  ASSERT_EQ(0.65, j2p_s.get_value());
  ASSERT_EQ(0.2, j2v_s.get_value());
  ASSERT_EQ(0.5, vo_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));
  ASSERT_TRUE(std::isnan(vo_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j2p_c.set_value(0.33);
  vo_c.set_value(0.99);

  // State values should not be changed
  ASSERT_EQ(1.55, j1p_s.get_value());
  ASSERT_EQ(0.1, j1v_s.get_value());
  ASSERT_EQ(0.65, j2p_s.get_value());
  ASSERT_EQ(0.2, j2v_s.get_value());
  ASSERT_EQ(0.5, vo_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.99, vo_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(1.55, j1p_s.get_value());
  ASSERT_EQ(0.1, j1v_s.get_value());
  ASSERT_EQ(0.65, j2p_s.get_value());
  ASSERT_EQ(0.2, j2v_s.get_value());
  ASSERT_EQ(0.5, vo_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.99, vo_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.1, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.99, vo_s.get_value());
  ASSERT_EQ(0.2, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.99, vo_c.get_value());
}

TEST_F(TestGenericSystem, generic_system_2dof_sensor)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_with_sensor_ +
              ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(8u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/fx"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/fy"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/tx"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/ty"));

  ASSERT_EQ(4u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_FALSE(rm.command_interface_exists("tcp_force_sensor/fx"));
  EXPECT_FALSE(rm.command_interface_exists("tcp_force_sensor/fy"));
  EXPECT_FALSE(rm.command_interface_exists("tcp_force_sensor/tx"));
  EXPECT_FALSE(rm.command_interface_exists("tcp_force_sensor/ty"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedStateInterface sfx_s = rm.claim_state_interface("tcp_force_sensor/fx");
  hardware_interface::LoanedStateInterface sfy_s = rm.claim_state_interface("tcp_force_sensor/fy");
  hardware_interface::LoanedStateInterface stx_s = rm.claim_state_interface("tcp_force_sensor/tx");
  hardware_interface::LoanedStateInterface sty_s = rm.claim_state_interface("tcp_force_sensor/ty");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  EXPECT_ANY_THROW(rm.claim_command_interface("tcp_force_sensor/fx"));
  EXPECT_ANY_THROW(rm.claim_command_interface("tcp_force_sensor/fy"));
  EXPECT_ANY_THROW(rm.claim_command_interface("tcp_force_sensor/tx"));
  EXPECT_ANY_THROW(rm.claim_command_interface("tcp_force_sensor/ty"));

  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j2p_c.set_value(0.33);

  // State values should not be changed
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
}

void test_generic_system_with_fake_sensor_commands(std::string urdf)
{
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(8u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/fx"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/fy"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/tx"));
  EXPECT_TRUE(rm.state_interface_exists("tcp_force_sensor/ty"));

  ASSERT_EQ(8u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("tcp_force_sensor/fx"));
  EXPECT_TRUE(rm.command_interface_exists("tcp_force_sensor/fy"));
  EXPECT_TRUE(rm.command_interface_exists("tcp_force_sensor/tx"));
  EXPECT_TRUE(rm.command_interface_exists("tcp_force_sensor/ty"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedStateInterface sfx_s = rm.claim_state_interface("tcp_force_sensor/fx");
  hardware_interface::LoanedStateInterface sfy_s = rm.claim_state_interface("tcp_force_sensor/fy");
  hardware_interface::LoanedStateInterface stx_s = rm.claim_state_interface("tcp_force_sensor/tx");
  hardware_interface::LoanedStateInterface sty_s = rm.claim_state_interface("tcp_force_sensor/ty");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface sfx_c =
    rm.claim_command_interface("tcp_force_sensor/fx");
  hardware_interface::LoanedCommandInterface sfy_c =
    rm.claim_command_interface("tcp_force_sensor/fy");
  hardware_interface::LoanedCommandInterface stx_c =
    rm.claim_command_interface("tcp_force_sensor/tx");
  hardware_interface::LoanedCommandInterface sty_c =
    rm.claim_command_interface("tcp_force_sensor/ty");

  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));
  EXPECT_TRUE(std::isnan(sfx_c.get_value()));
  EXPECT_TRUE(std::isnan(sfy_c.get_value()));
  EXPECT_TRUE(std::isnan(stx_c.get_value()));
  EXPECT_TRUE(std::isnan(sty_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j2p_c.set_value(0.33);
  sfx_c.set_value(1.11);
  sfy_c.set_value(2.22);
  stx_c.set_value(3.33);
  sty_c.set_value(4.44);

  // State values should not be changed
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(1.11, sfx_c.get_value());
  ASSERT_EQ(2.22, sfy_c.get_value());
  ASSERT_EQ(3.33, stx_c.get_value());
  ASSERT_EQ(4.44, sty_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(0.0, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  EXPECT_TRUE(std::isnan(sfx_s.get_value()));
  EXPECT_TRUE(std::isnan(sfy_s.get_value()));
  EXPECT_TRUE(std::isnan(stx_s.get_value()));
  EXPECT_TRUE(std::isnan(sty_s.get_value()));
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(1.11, sfx_c.get_value());
  ASSERT_EQ(2.22, sfy_c.get_value());
  ASSERT_EQ(3.33, stx_c.get_value());
  ASSERT_EQ(4.44, sty_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(1.11, sfx_s.get_value());
  ASSERT_EQ(2.22, sfy_s.get_value());
  ASSERT_EQ(3.33, stx_s.get_value());
  ASSERT_EQ(4.44, sty_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(1.11, sfx_c.get_value());
  ASSERT_EQ(2.22, sfy_c.get_value());
  ASSERT_EQ(3.33, stx_c.get_value());
  ASSERT_EQ(4.44, sty_c.get_value());
}

TEST_F(TestGenericSystem, generic_system_2dof_sensor_fake_command)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_with_sensor_fake_command_ +
              ros2_control_test_assets::urdf_tail;

  test_generic_system_with_fake_sensor_commands(urdf);
}

TEST_F(TestGenericSystem, generic_system_2dof_sensor_fake_command_True)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system_2dof_with_sensor_fake_command_True_ +
              ros2_control_test_assets::urdf_tail;

  test_generic_system_with_fake_sensor_commands(urdf);
}

void test_generic_system_with_mimic_joint(std::string urdf)
{
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(4u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));

  ASSERT_EQ(4u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j1v_c = rm.claim_command_interface("joint1/velocity");

  ASSERT_EQ(1.57, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j1v_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j1v_c.set_value(0.05);

  // State values should not be changed
  ASSERT_EQ(1.57, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.05, j1v_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(1.57, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(0.0, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.05, j1v_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.05, j1v_s.get_value());
  ASSERT_EQ(-0.22, j2p_s.get_value());
  ASSERT_EQ(-0.1, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.05, j1v_c.get_value());
}

TEST_F(TestGenericSystem, hardware_system_2dof_with_mimic_joint)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_with_mimic_joint_ +
              ros2_control_test_assets::urdf_tail;

  test_generic_system_with_mimic_joint(urdf);
}

TEST_F(TestGenericSystem, generic_system_2dof_functionality_with_offset)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system_2dof_standard_interfaces_with_offset_ +
              ros2_control_test_assets::urdf_tail;

  generic_system_functional_test(urdf, -3);
}

TEST_F(TestGenericSystem, generic_system_2dof_functionality_with_offset_custom_interface_missing)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_missing_ +
              ros2_control_test_assets::urdf_tail;

  // custom interface is missing so offset will not be applied
  generic_system_functional_test(urdf, 0.0);
}

TEST_F(TestGenericSystem, generic_system_2dof_functionality_with_offset_custom_interface)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system_2dof_standard_interfaces_with_custom_interface_for_offset_ +
              ros2_control_test_assets::urdf_tail;

  const double offset = -3;

  hardware_interface::ResourceManager rm(urdf);

  // check is hardware is configured
  auto states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1v_s = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2v_s = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j1v_c = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j2v_c = rm.claim_command_interface("joint2/velocity");

  // set default value of custom state interfaces to anything first
  hardware_interface::LoanedStateInterface c_j1p_s =
    rm.claim_state_interface("joint1/actual_position");
  hardware_interface::LoanedStateInterface c_j2p_s =
    rm.claim_state_interface("joint2/actual_position");

  // State interfaces without initial value are set to 0
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  ASSERT_TRUE(std::isnan(j1v_c.get_value()));
  ASSERT_TRUE(std::isnan(j2p_c.get_value()));
  ASSERT_TRUE(std::isnan(j2v_c.get_value()));

  // set some new values in commands
  j1p_c.set_value(0.11);
  j1v_c.set_value(0.22);
  j2p_c.set_value(0.33);
  j2v_c.set_value(0.44);

  // State values should not be changed
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_EQ(3.45, j1p_s.get_value());
  ASSERT_EQ(0.0, j1v_s.get_value());
  ASSERT_EQ(2.78, j2p_s.get_value());
  ASSERT_EQ(0.0, j2v_s.get_value());
  ASSERT_EQ(0.11, j1p_c.get_value());
  ASSERT_EQ(0.22, j1v_c.get_value());
  ASSERT_EQ(0.33, j2p_c.get_value());
  ASSERT_EQ(0.44, j2v_c.get_value());

  // read() mirrors commands + offset to states
  rm.read();
  ASSERT_EQ(0.11, j1p_s.get_value());
  ASSERT_EQ(0.11 + offset, c_j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.33 + offset, c_j2p_s.get_value());
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
  ASSERT_EQ(0.11 + offset, c_j1p_s.get_value());
  ASSERT_EQ(0.22, j1v_s.get_value());
  ASSERT_EQ(0.33, j2p_s.get_value());
  ASSERT_EQ(0.33 + offset, c_j2p_s.get_value());
  ASSERT_EQ(0.44, j2v_s.get_value());
  ASSERT_EQ(0.55, j1p_c.get_value());
  ASSERT_EQ(0.66, j1v_c.get_value());
  ASSERT_EQ(0.77, j2p_c.get_value());
  ASSERT_EQ(0.88, j2v_c.get_value());

  rm.start_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::ACTIVE);

  rm.stop_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

TEST_F(TestGenericSystem, valid_urdf_ros2_control_system_robot_with_gpio_)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              valid_urdf_ros2_control_system_robot_with_gpio_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // check is hardware is configured
  std::unordered_map<std::string, rclcpp_lifecycle::State> states_map;
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);

  ASSERT_EQ(8u, rm.state_interface_keys().size());
  ASSERT_EQ(6, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_output1"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_input1"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_input2"));
  EXPECT_TRUE(rm.state_interface_exists("flange_vacuum/vacuum"));

  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("flange_analog_IOs/analog_output1"));
  EXPECT_TRUE(rm.command_interface_exists("flange_vacuum/vacuum"));

  // Check initial values
  hardware_interface::LoanedStateInterface gpio1_a_o1_s =
    rm.claim_state_interface("flange_analog_IOs/analog_output1");
  hardware_interface::LoanedStateInterface gpio1_a_i1_s =
    rm.claim_state_interface("flange_analog_IOs/analog_input1");
  hardware_interface::LoanedStateInterface gpio1_a_o2_s =
    rm.claim_state_interface("flange_analog_IOs/analog_input2");
  hardware_interface::LoanedStateInterface gpio2_vac_s =
    rm.claim_state_interface("flange_vacuum/vacuum");
  hardware_interface::LoanedCommandInterface gpio1_a_o1_c =
    rm.claim_command_interface("flange_analog_IOs/analog_output1");
  hardware_interface::LoanedCommandInterface gpio2_vac_c =
    rm.claim_command_interface("flange_vacuum/vacuum");

  // State interfaces without initial value are set to 0
  ASSERT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  ASSERT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  ASSERT_TRUE(std::isnan(gpio1_a_o1_c.get_value()));
  ASSERT_TRUE(std::isnan(gpio2_vac_c.get_value()));

  // set some new values in commands
  gpio1_a_o1_c.set_value(0.111);
  gpio2_vac_c.set_value(0.222);

  // State values should not be changed
  ASSERT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  ASSERT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  ASSERT_EQ(0.111, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.222, gpio2_vac_c.get_value());

  // write() does not change values
  rm.write();
  ASSERT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  ASSERT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  ASSERT_EQ(0.111, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.222, gpio2_vac_c.get_value());

  // read() mirrors commands + offset to states
  rm.read();
  ASSERT_EQ(0.111, gpio1_a_o1_s.get_value());
  ASSERT_EQ(0.222, gpio2_vac_s.get_value());
  ASSERT_EQ(0.111, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.222, gpio2_vac_c.get_value());

  // set some new values in commands
  gpio1_a_o1_c.set_value(0.333);
  gpio2_vac_c.set_value(0.444);

  // state values should not be changed
  ASSERT_EQ(0.111, gpio1_a_o1_s.get_value());
  ASSERT_EQ(0.222, gpio2_vac_s.get_value());
  ASSERT_EQ(0.333, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.444, gpio2_vac_c.get_value());

  rm.start_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::ACTIVE);

  rm.stop_components();
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);

  // check other functionalities are working well
  generic_system_functional_test(urdf);
}

TEST_F(TestGenericSystem, valid_urdf_ros2_control_system_robot_with_gpio_fake_command_)
{
  auto urdf = ros2_control_test_assets::urdf_head +
              valid_urdf_ros2_control_system_robot_with_gpio_fake_command_ +
              ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // check is hardware is configured
  std::unordered_map<std::string, rclcpp_lifecycle::State> states_map;
  states_map = rm.get_components_states();
  EXPECT_EQ(
    states_map["GenericSystem2dof"].label(), hardware_interface::lifecycle_state_names::INACTIVE);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(8u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_output1"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_input1"));
  EXPECT_TRUE(rm.state_interface_exists("flange_analog_IOs/analog_input2"));
  EXPECT_TRUE(rm.state_interface_exists("flange_vacuum/vacuum"));

  ASSERT_EQ(8u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("flange_analog_IOs/analog_output1"));
  EXPECT_TRUE(rm.command_interface_exists("flange_analog_IOs/analog_input1"));
  EXPECT_TRUE(rm.command_interface_exists("flange_analog_IOs/analog_input2"));
  EXPECT_TRUE(rm.command_interface_exists("flange_vacuum/vacuum"));

  // Check initial values
  hardware_interface::LoanedStateInterface gpio1_a_o1_s =
    rm.claim_state_interface("flange_analog_IOs/analog_output1");
  hardware_interface::LoanedStateInterface gpio1_a_i1_s =
    rm.claim_state_interface("flange_analog_IOs/analog_input1");
  hardware_interface::LoanedStateInterface gpio1_a_o2_s =
    rm.claim_state_interface("flange_analog_IOs/analog_input2");
  hardware_interface::LoanedStateInterface gpio2_vac_s =
    rm.claim_state_interface("flange_vacuum/vacuum");
  hardware_interface::LoanedCommandInterface gpio1_a_o1_c =
    rm.claim_command_interface("flange_analog_IOs/analog_output1");
  hardware_interface::LoanedCommandInterface gpio1_a_i1_c =
    rm.claim_command_interface("flange_analog_IOs/analog_input1");
  hardware_interface::LoanedCommandInterface gpio1_a_i2_c =
    rm.claim_command_interface("flange_analog_IOs/analog_input2");
  hardware_interface::LoanedCommandInterface gpio2_vac_c =
    rm.claim_command_interface("flange_vacuum/vacuum");

  EXPECT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_i1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_o2_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_o1_c.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_i1_c.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_i2_c.get_value()));
  EXPECT_TRUE(std::isnan(gpio2_vac_c.get_value()));

  // set some new values in commands
  gpio1_a_o1_c.set_value(0.11);
  gpio1_a_i1_c.set_value(0.33);
  gpio1_a_i2_c.set_value(1.11);
  gpio2_vac_c.set_value(2.22);

  // State values should not be changed
  EXPECT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_i1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_o2_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  ASSERT_EQ(0.11, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.33, gpio1_a_i1_c.get_value());
  ASSERT_EQ(1.11, gpio1_a_i2_c.get_value());
  ASSERT_EQ(2.22, gpio2_vac_c.get_value());

  // write() does not change values
  rm.write();
  EXPECT_TRUE(std::isnan(gpio1_a_o1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_i1_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio1_a_o2_s.get_value()));
  EXPECT_TRUE(std::isnan(gpio2_vac_s.get_value()));
  ASSERT_EQ(0.11, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.33, gpio1_a_i1_c.get_value());
  ASSERT_EQ(1.11, gpio1_a_i2_c.get_value());
  ASSERT_EQ(2.22, gpio2_vac_c.get_value());

  // read() mirrors commands to states
  rm.read();
  ASSERT_EQ(0.11, gpio1_a_o1_s.get_value());
  ASSERT_EQ(0.33, gpio1_a_i1_s.get_value());
  ASSERT_EQ(1.11, gpio1_a_o2_s.get_value());
  ASSERT_EQ(2.22, gpio2_vac_s.get_value());
  ASSERT_EQ(0.11, gpio1_a_o1_c.get_value());
  ASSERT_EQ(0.33, gpio1_a_i1_c.get_value());
  ASSERT_EQ(1.11, gpio1_a_i2_c.get_value());
  ASSERT_EQ(2.22, gpio2_vac_c.get_value());
}
