// Copyright 2021 Department of Engineering Cybernetics, NTNU.
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
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller_with_interfaces/test_controller_with_interfaces.hpp"

using ::testing::_;
using ::testing::Return;

class TestReleaseInterfaces : public ControllerManagerFixture<controller_manager::ControllerManager>
{
};

TEST_F(TestReleaseInterfaces, switch_controllers_same_interface)
{
  std::string controller_type =
    test_controller_with_interfaces::TEST_CONTROLLER_WITH_INTERFACES_CLASS_NAME;

  // Load two controllers of different names
  std::string controller_name1 = "test_controller1";
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  ASSERT_NO_THROW(cm_->load_controller(controller_name2, controller_type));
  ASSERT_EQ(2u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 = cm_->get_loaded_controllers()[0];
  controller_manager::ControllerSpec abstract_test_controller2 = cm_->get_loaded_controllers()[1];

  // Configure controllers
  ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(controller_name1));
  ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(controller_name2));

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2.c->get_lifecycle_state().id());

  {  // Test starting the first controller
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #1");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test starting the second controller when the first is running
    // Fails as they have the same command interface
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #2");
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping controller #1 and starting controller #2
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1 and starting controller #2");
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {controller_name1};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping controller #2 and starting controller #1
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #1 and stopping controller #2");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping both controllers when only controller #1 is running
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping both controllers (will fail in STRICT mode as controller #2 is not running)");
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1, controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    // The call to switch above will fail and the CM will not block
    // ASSERT_EQ(
    //   std::future_status::timeout,
    //   switch_future.wait_for(std::chrono::milliseconds(100))) <<
    //   "switch_controller should be blocking until next update cycle";
    // ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::ERROR, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping both controllers when only controller #1 is running
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping both controllers (will fail in BEST EFFORT mode as controller #2 is not running)");
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1, controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, BEST_EFFORT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test starting both controllers at the same time
    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting both controllers at the same time (should notify about resource conflict)");
    std::vector<std::string> start_controllers = {controller_name1, controller_name2};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }
}
