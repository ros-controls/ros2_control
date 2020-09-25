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
