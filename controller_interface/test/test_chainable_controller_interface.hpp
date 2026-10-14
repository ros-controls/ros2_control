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

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/handle.hpp"

constexpr char TEST_CONTROLLER_NAME[] = "testable_chainable_controller";
constexpr double INTERFACE_VALUE = 1989.0;
constexpr double INTERFACE_VALUE_SUBSCRIBER_ERROR = 12345.0;
constexpr double INTERFACE_VALUE_UPDATE_ERROR = 67890.0;
constexpr double INTERFACE_VALUE_INITIAL_REF = 1984.0;
constexpr double EXPORTED_STATE_INTERFACE_VALUE = 21833.0;
constexpr double EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE = 82802.0;

class TestableChainableControllerInterface
: public controller_interface::ChainableControllerInterface
{
public:
  FRIEND_TEST(ChainableControllerInterfaceTest, export_state_interfaces);
  FRIEND_TEST(ChainableControllerInterfaceTest, export_reference_interfaces);
  FRIEND_TEST(ChainableControllerInterfaceTest, test_update_logic);

  controller_interface::CallbackReturn on_init() override
  {
    name_prefix_of_interfaces_ = get_node()->get_name();
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

  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list()
    override
  {
    state_itf_ptr_ = std::make_shared<hardware_interface::StateInterface>(
      name_prefix_of_interfaces_, "test_state");
    std::ignore = state_itf_ptr_->set_value(EXPORTED_STATE_INTERFACE_VALUE);
    return {state_itf_ptr_};
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    ref_itf_ptr_ = std::make_shared<hardware_interface::CommandInterface>(
      name_prefix_of_interfaces_, "test_itf");
    std::ignore = ref_itf_ptr_->set_value(INTERFACE_VALUE);
    return {ref_itf_ptr_};
  }

  bool on_set_chained_mode(bool /*chained_mode*/) override
  {
    const auto ref_val = ref_itf_ptr_->get_optional();
    if (ref_val.has_value() && ref_val.value() == 0.0)
    {
      std::ignore = state_itf_ptr_->set_value(EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE);
      return true;
    }
    return false;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (reference_interface_value_ == INTERFACE_VALUE_SUBSCRIBER_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }
    std::ignore = ref_itf_ptr_->set_value(reference_interface_value_);
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const auto ref_val = ref_itf_ptr_->get_optional().value();
    if (ref_val == INTERFACE_VALUE_UPDATE_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }
    std::ignore = ref_itf_ptr_->set_value(ref_val - 1.0);
    std::ignore = state_itf_ptr_->set_value(state_itf_ptr_->get_optional().value() + 1.0);
    return controller_interface::return_type::OK;
  }

  void set_name_prefix_of_reference_interfaces(const std::string & prefix)
  {
    name_prefix_of_interfaces_ = prefix;
  }

  void set_new_reference_interface_value(const double ref_itf_value)
  {
    reference_interface_value_ = ref_itf_value;
  }

  hardware_interface::CommandInterface::SharedPtr ref_itf_ptr_;
  hardware_interface::StateInterface::SharedPtr state_itf_ptr_;
  std::string name_prefix_of_interfaces_;
  double reference_interface_value_ = INTERFACE_VALUE_INITIAL_REF;
};

class ChainableControllerInterfaceTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

#endif  // TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
