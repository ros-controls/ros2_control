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
#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"
#include "./test_controller/test_controller.hpp"

using namespace std::chrono_literals;

TEST_F(TestControllerManager, controller_lifecycle) {
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    robot_, executor_,
    "test_controller_manager");


  auto test_controller = std::make_shared<test_controller::TestController>();
  cm->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);
  EXPECT_EQ(1u, cm->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(
    0u,
    test_controller->internal_counter) <<
    "Update should not reach an unstarted controller";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";

  // Start controller, will take effect at the end of the update function
  std::vector<std::string> start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  std::vector<std::string> stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async,
    &controller_manager::ControllerManager::switch_controller, cm,
    start_controllers, stop_controllers,
    STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(
    std::future_status::timeout,
    switch_future.wait_for(std::chrono::milliseconds(100))) <<
    "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is started at the end of update";
  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    switch_future.get()
  );
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(1u, test_controller->internal_counter);

  // Stop controller, will take effect at the end of the update function
  start_controllers = {};
  stop_controllers = {test_controller::TEST_CONTROLLER_NAME};
  switch_future = std::async(
    std::launch::async,
    &controller_manager::ControllerManager::switch_controller, cm,
    start_controllers, stop_controllers,
    STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(
    std::future_status::timeout,
    switch_future.wait_for(std::chrono::milliseconds(100))) <<
    "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(
    2u,
    test_controller->internal_counter) <<
    "Controller is stopped at the end of update, so it should have done one more update";
  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    switch_future.get()
  );

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());
  auto unload_future = std::async(
    std::launch::async,
    &controller_manager::ControllerManager::unload_controller, cm,
    test_controller::TEST_CONTROLLER_NAME);

  ASSERT_EQ(
    std::future_status::timeout,
    unload_future.wait_for(std::chrono::milliseconds(100))) <<
    "unload_controller should be blocking until next update cycle";
  cm->update();
  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    unload_future.get()
  );

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_node()->get_current_state().id());
  EXPECT_EQ(1, test_controller.use_count());
}

TEST_F(TestControllerManager, controller_parameters) {
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    robot_, executor_,
    "test_controller_manager");

  auto executor_spin_future = std::async(
    std::launch::async, [this]() -> void {
      executor_->spin();
    });
  // This sleep is needed to prevent a too fast test from ending before the
  // executor has began to spin, which causes it to hang
  std::this_thread::sleep_for(50ms);

  // controller_manager_parameter because it has no prefix
  // (and controller manager accepts all parameters)
  rclcpp::Parameter controller_manager_parameter("double_param", 0.5);
  rclcpp::Parameter test_controller_parameter(
    test_controller::TEST_CONTROLLER_NAME + std::string(".int_param"), 123);

  EXPECT_TRUE(cm->set_parameter(controller_manager_parameter).successful);
  EXPECT_TRUE(cm->set_parameter(test_controller_parameter).successful);

  auto test_controller = std::make_shared<test_controller::TestController>();
  cm->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_TYPE);

  std::string string_param;
  int int_param;
  double double_param;
  // Default parameters are still present, except the one we set before loading the controller
  EXPECT_TRUE(test_controller->get_lifecycle_node()->get_parameter("string_param", string_param));
  EXPECT_TRUE(test_controller->get_lifecycle_node()->get_parameter("int_param", int_param));
  EXPECT_TRUE(test_controller->get_lifecycle_node()->get_parameter("double_param", double_param));
  EXPECT_EQ(string_param, test_controller::DEFAULT_STR_PARAM_VALUE) <<
    "parameter should have default value";
  EXPECT_EQ(int_param, 123) <<
    "parameter should have updated value set by controller manager on load";
  EXPECT_DOUBLE_EQ(double_param, test_controller::DEFAULT_DOUBLE_PARAM_VALUE) <<
    "parameter should have default value";

  // Update the controllers' parameters by setting the controller_manager parameters
  rclcpp::Parameter runtime_changed_controller_param(
    test_controller::TEST_CONTROLLER_NAME + std::string(".string_param"), "runtime changed");
  EXPECT_TRUE(cm->set_parameter(runtime_changed_controller_param).successful);
  EXPECT_TRUE(test_controller->get_lifecycle_node()->get_parameter("string_param", string_param));
  EXPECT_EQ(string_param, "runtime changed");

  // Attempt to set an invalid controller parameter
  rclcpp::Parameter invalid_test_controller_parameter(
    test_controller::TEST_CONTROLLER_NAME + std::string(".invalid_param"), 123);
  EXPECT_ANY_THROW(cm->set_parameter(invalid_test_controller_parameter));

  executor_->cancel();
}

TEST_F(TestControllerManager, controller_invalid_load_parameters) {
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    robot_, executor_,
    "test_controller_manager");

  rclcpp::Parameter invalid_controller_parameter(
    test_controller::TEST_CONTROLLER_NAME + std::string(".invalid_parameter"), 123);

  EXPECT_TRUE(cm->set_parameter(invalid_controller_parameter).successful);

  auto test_controller = std::make_shared<test_controller::TestController>();
  ASSERT_FALSE(
    cm->add_controller(
      test_controller, test_controller::TEST_CONTROLLER_NAME,
      test_controller::TEST_CONTROLLER_TYPE).get());
}
