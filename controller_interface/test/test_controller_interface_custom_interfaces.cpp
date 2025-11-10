// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschraenkt)
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
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

class TestableControllerInterfaceCustom : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  // Expose protected members for testing
  using controller_interface::ControllerInterfaceBase::command_interface_types_;
  using controller_interface::ControllerInterfaceBase::state_interface_types_;
};

class TestControllerInterfaceCustomInterfaces : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestControllerInterfaceCustomInterfaces, custom_interfaces_type_tracking)
{
  // Create interface descriptions for different data types
  hardware_interface::InterfaceInfo cmd_bool_info;
  cmd_bool_info.name = "command1";
  cmd_bool_info.data_type = "bool";
  hardware_interface::InterfaceDescription cmd_bool_descr("joint1", cmd_bool_info);

  hardware_interface::InterfaceInfo cmd_int32_info;
  cmd_int32_info.name = "command2";
  cmd_int32_info.data_type = "int32";
  hardware_interface::InterfaceDescription cmd_int32_descr("joint1", cmd_int32_info);

  hardware_interface::InterfaceInfo cmd_int16_info;
  cmd_int16_info.name = "command3";
  cmd_int16_info.data_type = "int16";
  hardware_interface::InterfaceDescription cmd_int16_descr("joint1", cmd_int16_info);

  hardware_interface::InterfaceInfo cmd_uint16_info;
  cmd_uint16_info.name = "command4";
  cmd_uint16_info.data_type = "uint16";
  hardware_interface::InterfaceDescription cmd_uint16_descr("joint1", cmd_uint16_info);

  hardware_interface::InterfaceInfo cmd_uint32_info;
  cmd_uint32_info.name = "command5";
  cmd_uint32_info.data_type = "uint32";
  hardware_interface::InterfaceDescription cmd_uint32_descr("joint1", cmd_uint32_info);

  hardware_interface::InterfaceInfo cmd_int64_info;
  cmd_int64_info.name = "command6";
  cmd_int64_info.data_type = "int64";
  hardware_interface::InterfaceDescription cmd_int64_descr("joint1", cmd_int64_info);

  hardware_interface::InterfaceInfo cmd_uint64_info;
  cmd_uint64_info.name = "command7";
  cmd_uint64_info.data_type = "uint64";
  hardware_interface::InterfaceDescription cmd_uint64_descr("joint1", cmd_uint64_info);

  hardware_interface::InterfaceInfo state_bool_info;
  state_bool_info.name = "state1";
  state_bool_info.data_type = "bool";
  hardware_interface::InterfaceDescription state_bool_descr("joint1", state_bool_info);

  hardware_interface::InterfaceInfo state_int32_info;
  state_int32_info.name = "state2";
  state_int32_info.data_type = "int32";
  hardware_interface::InterfaceDescription state_int32_descr("joint1", state_int32_info);

  hardware_interface::InterfaceInfo state_int16_info;
  state_int16_info.name = "state3";
  state_int16_info.data_type = "int16";
  hardware_interface::InterfaceDescription state_int16_descr("joint1", state_int16_info);

  hardware_interface::InterfaceInfo state_uint16_info;
  state_uint16_info.name = "state4";
  state_uint16_info.data_type = "uint16";
  hardware_interface::InterfaceDescription state_uint16_descr("joint1", state_uint16_info);

  hardware_interface::InterfaceInfo state_uint32_info;
  state_uint32_info.name = "state5";
  state_uint32_info.data_type = "uint32";
  hardware_interface::InterfaceDescription state_uint32_descr("joint1", state_uint32_info);

  hardware_interface::InterfaceInfo state_int64_info;
  state_int64_info.name = "state6";
  state_int64_info.data_type = "int64";
  hardware_interface::InterfaceDescription state_int64_descr("joint1", state_int64_info);

  hardware_interface::InterfaceInfo state_uint64_info;
  state_uint64_info.name = "state7";
  state_uint64_info.data_type = "uint64";
  hardware_interface::InterfaceDescription state_uint64_descr("joint1", state_uint64_info);

  // Create command and state interfaces
  auto cmd_bool = std::make_shared<hardware_interface::CommandInterface>(cmd_bool_descr);
  auto cmd_int32 = std::make_shared<hardware_interface::CommandInterface>(cmd_int32_descr);
  auto cmd_int16 = std::make_shared<hardware_interface::CommandInterface>(cmd_int16_descr);
  auto cmd_uint16 = std::make_shared<hardware_interface::CommandInterface>(cmd_uint16_descr);
  auto cmd_uint32 = std::make_shared<hardware_interface::CommandInterface>(cmd_uint32_descr);
  auto cmd_int64 = std::make_shared<hardware_interface::CommandInterface>(cmd_int64_descr);
  auto cmd_uint64 = std::make_shared<hardware_interface::CommandInterface>(cmd_uint64_descr);
  auto state_bool = std::make_shared<hardware_interface::StateInterface>(state_bool_descr);
  auto state_int32 = std::make_shared<hardware_interface::StateInterface>(state_int32_descr);
  auto state_int16 = std::make_shared<hardware_interface::StateInterface>(state_int16_descr);
  auto state_uint16 = std::make_shared<hardware_interface::StateInterface>(state_uint16_descr);
  auto state_uint32 = std::make_shared<hardware_interface::StateInterface>(state_uint32_descr);
  auto state_int64 = std::make_shared<hardware_interface::StateInterface>(state_int64_descr);
  auto state_uint64 = std::make_shared<hardware_interface::StateInterface>(state_uint64_descr);

  // Build loaned interface vectors
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(cmd_bool);
  command_interfaces.emplace_back(cmd_int32);
  command_interfaces.emplace_back(cmd_int16);
  command_interfaces.emplace_back(cmd_uint16);
  command_interfaces.emplace_back(cmd_uint32);
  command_interfaces.emplace_back(cmd_int64);
  command_interfaces.emplace_back(cmd_uint64);

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(state_bool);
  state_interfaces.emplace_back(state_int32);
  state_interfaces.emplace_back(state_int16);
  state_interfaces.emplace_back(state_uint16);
  state_interfaces.emplace_back(state_uint32);
  state_interfaces.emplace_back(state_int64);
  state_interfaces.emplace_back(state_uint64);

  // Create a controller and initialize it
  auto controller = std::make_shared<TestableControllerInterfaceCustom>();
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_controller_custom";
  params.robot_description = "";
  params.update_rate = 100;
  params.node_namespace = "";
  params.node_options = controller->define_custom_node_options();
  ASSERT_EQ(controller->init(params), controller_interface::return_type::OK);

  // Assign interfaces
  controller->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  // Verify the type map is populated correctly
  EXPECT_EQ(controller->command_interface_types_["joint1/command1"], "bool");
  EXPECT_EQ(controller->command_interface_types_["joint1/command2"], "int32");
  EXPECT_EQ(controller->command_interface_types_["joint1/command3"], "int16");
  EXPECT_EQ(controller->command_interface_types_["joint1/command4"], "uint16");
  EXPECT_EQ(controller->command_interface_types_["joint1/command5"], "uint32");
  EXPECT_EQ(controller->command_interface_types_["joint1/command6"], "int64");
  EXPECT_EQ(controller->command_interface_types_["joint1/command7"], "uint64");
  EXPECT_EQ(controller->state_interface_types_["joint1/state1"], "bool");
  EXPECT_EQ(controller->state_interface_types_["joint1/state2"], "int32");
  EXPECT_EQ(controller->state_interface_types_["joint1/state3"], "int16");
  EXPECT_EQ(controller->state_interface_types_["joint1/state4"], "uint16");
  EXPECT_EQ(controller->state_interface_types_["joint1/state5"], "uint32");
  EXPECT_EQ(controller->state_interface_types_["joint1/state6"], "int64");
  EXPECT_EQ(controller->state_interface_types_["joint1/state7"], "uint64");

  // Release interfaces and verify maps are cleared
  controller->release_interfaces();
  EXPECT_TRUE(controller->command_interface_types_.empty());
  EXPECT_TRUE(controller->state_interface_types_.empty());

  controller->get_node()->shutdown();
}
