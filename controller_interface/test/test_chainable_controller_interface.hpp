// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
#define TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_

#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

constexpr char TEST_CONTROLLER_NAME[] = "testable_chainable_controller";
constexpr double INTERFACE_VALUE = 1989.0;
constexpr double INTERFACE_VALUE_SUBSCRIBER_ERROR = 12345.0;
constexpr double INTERFACE_VALUE_UPDATE_ERROR = 67890.0;
constexpr double INTERFACE_VALUE_INITIAL_REF = 1984.0;

class TestableChainableControllerInterface
: public controller_interface::ChainableControllerInterface
{
public:
  FRIEND_TEST(ChainableControllerInterfaceTest, reference_interfaces_storage_not_correct_size);
  FRIEND_TEST(ChainableControllerInterfaceTest, test_update_logic);

  TestableChainableControllerInterface()
  {
    reference_interfaces_.reserve(1);
    reference_interfaces_.push_back(INTERFACE_VALUE);
  }

  controller_interface::CallbackReturn on_init() override
  {
    // set default value
    name_prefix_of_reference_interfaces_ = get_node()->get_name();

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

  // Implementation of ChainableController virtual methods
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.push_back(hardware_interface::CommandInterface(
      name_prefix_of_reference_interfaces_, "test_itf", &reference_interfaces_[0]));

    return command_interfaces;
  }

  bool on_set_chained_mode(bool /*chained_mode*/) override
  {
    if (reference_interfaces_[0] == 0.0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  controller_interface::return_type update_reference_from_subscribers() override
  {
    if (reference_interface_value_ == INTERFACE_VALUE_SUBSCRIBER_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }

    reference_interfaces_[0] = reference_interface_value_;
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (reference_interfaces_[0] == INTERFACE_VALUE_UPDATE_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }

    reference_interfaces_[0] -= 1;

    return controller_interface::return_type::OK;
  }

  void set_name_prefix_of_reference_interfaces(const std::string & prefix)
  {
    name_prefix_of_reference_interfaces_ = prefix;
  }

  void set_new_reference_interface_value(const double ref_itf_value)
  {
    reference_interface_value_ = ref_itf_value;
  }

  std::string name_prefix_of_reference_interfaces_;
  double reference_interface_value_ = INTERFACE_VALUE_INITIAL_REF;
};

class ChainableControllerInterfaceTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

#endif  // TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
