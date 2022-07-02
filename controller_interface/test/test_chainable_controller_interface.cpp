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

#include "test_chainable_controller_interface.hpp"

#include <gmock/gmock.h>

TEST_F(ChainableControllerInterfaceTest, default_returns)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  EXPECT_TRUE(controller.is_chainable());
  EXPECT_FALSE(controller.is_in_chained_mode());
}

TEST_F(ChainableControllerInterfaceTest, export_reference_interfaces)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();

  ASSERT_EQ(reference_interfaces.size(), 1u);
  EXPECT_EQ(reference_interfaces[0].get_prefix_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(reference_interfaces[0].get_interface_name(), "test_itf");

  EXPECT_EQ(reference_interfaces[0].get_value(), INTERFACE_VALUE);
}

TEST_F(ChainableControllerInterfaceTest, reference_interfaces_storage_not_correct_size)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  // expect empty return because storage is not resized
  controller.reference_interfaces_.clear();
  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces.empty());
}

TEST_F(ChainableControllerInterfaceTest, reference_interfaces_prefix_is_not_node_name)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  controller.set_name_prefix_of_reference_interfaces("some_not_correct_interface_prefix");

  // expect empty return because interface prefix is not equal to the node name
  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces.empty());
}

TEST_F(ChainableControllerInterfaceTest, setting_chained_mode)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 1u);

  EXPECT_FALSE(controller.is_in_chained_mode());

  // Fail setting chained mode
  EXPECT_EQ(reference_interfaces[0].get_value(), INTERFACE_VALUE);

  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  EXPECT_FALSE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  // Success setting chained mode
  reference_interfaces[0].set_value(0.0);

  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  controller.configure();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->activate();
  // Can not change chained mode until in "ACTIVE" state
  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->deactivate();
  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  // Can change 'chained' mode only in "UNCONFIGURED" state
  controller.get_node()->cleanup();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());
}

TEST_F(ChainableControllerInterfaceTest, test_update_logic)
{
  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  EXPECT_FALSE(controller.is_in_chained_mode());

  // call update and update it from subscriber because not in chained mode
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(controller.reference_interfaces_[0], INTERFACE_VALUE_INITIAL_REF - 1);

  // Provoke error in update from subscribers - return ERROR and update_and_write_commands not exec.
  controller.set_new_reference_interface_value(INTERFACE_VALUE_SUBSCRIBER_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(controller.reference_interfaces_[0], INTERFACE_VALUE_INITIAL_REF - 1);

  // Provoke error from update - return ERROR, but reference interface is updated and not reduced
  controller.set_new_reference_interface_value(INTERFACE_VALUE_UPDATE_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(controller.reference_interfaces_[0], INTERFACE_VALUE_UPDATE_ERROR);

  controller.reference_interfaces_[0] = 0.0;

  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  // Provoke error in update from subscribers - return OK because update of subscribers is not used
  // reference interface is not updated (updated directly because in chained mode)
  controller.set_new_reference_interface_value(INTERFACE_VALUE_SUBSCRIBER_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(controller.reference_interfaces_[0], -1.0);

  // Provoke error from update - return ERROR, but reference interface is updated directly
  controller.set_new_reference_interface_value(INTERFACE_VALUE_SUBSCRIBER_ERROR);
  controller.reference_interfaces_[0] = INTERFACE_VALUE_UPDATE_ERROR;
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(controller.reference_interfaces_[0], INTERFACE_VALUE_UPDATE_ERROR);
}
