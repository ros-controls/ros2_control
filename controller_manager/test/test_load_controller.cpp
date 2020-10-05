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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_test_common.hpp"

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/controller_manager.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"
using ::testing::_;
using ::testing::Return;

TEST_F(TestControllerManager, load_unknown_controller)
{
  controller_manager::ControllerManager cm(resource_manager_, executor_, "test_controller_manager");
  ASSERT_THROW(
    cm.load_controller("unknown_controller_name", "unknown_controller_type"), std::runtime_error);
}

TEST_F(TestControllerManager, load1_known_controller)
{
  controller_manager::ControllerManager cm(resource_manager_, executor_, "test_controller_manager");
  ASSERT_NO_THROW(cm.load_controller("test_controller_01", test_controller::TEST_CONTROLLER_TYPE));
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());

  controller_manager::ControllerSpec abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller.c->get_lifecycle_node();

  EXPECT_STREQ(lifecycle_node->get_name(), "test_controller_01");

  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller.c->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, load2_known_controller)
{
  controller_manager::ControllerManager cm(resource_manager_, executor_, "test_controller_manager");
  std::string controller_type = test_controller::TEST_CONTROLLER_TYPE;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm.load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm.get_loaded_controllers()[0];
  EXPECT_STREQ(
    controller_name1.c_str(), abstract_test_controller1.c->get_lifecycle_node()->get_name());
  abstract_test_controller1.c->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());

  // load the same controller again with a different name
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm.load_controller(controller_name2, controller_type));
  EXPECT_EQ(2u, cm.get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller2 =
    cm.get_loaded_controllers()[1];
  EXPECT_STREQ(
    controller_name2.c_str(), abstract_test_controller2.c->get_lifecycle_node()->get_name());
  EXPECT_STREQ(
    controller_name2.c_str(), abstract_test_controller2.info.name.c_str());
  abstract_test_controller2.c->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2.c->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, update)
{
  controller_manager::ControllerManager cm(resource_manager_, executor_, "test_controller_manager");
  ASSERT_NO_THROW(cm.load_controller("test_controller_01", test_controller::TEST_CONTROLLER_TYPE));

  controller_manager::ControllerSpec abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller.c->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller.c->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, register_controller_loader)
{
  controller_manager::ControllerManager cm(resource_manager_, executor_, "test_controller_manager");

  std::shared_ptr<ControllerLoaderMock> mock_loader(new ControllerLoaderMock);
  std::shared_ptr<ControllerMock> mock_controller(new ControllerMock);

  cm.register_controller_loader(mock_loader);

  const std::string mock_controller_name = "mock_controller_01";
  const std::string mock_controller_type = "mock_controller";

  EXPECT_CALL(*mock_loader, is_available(mock_controller_type))
  .Times(1)
  .WillOnce(Return(true));

  EXPECT_CALL(*mock_loader, create(mock_controller_type))
  .Times(1)
  .WillOnce(Return(mock_controller));

  ASSERT_NO_THROW(cm.load_controller(mock_controller_name, mock_controller_type));
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());

  controller_manager::ControllerSpec abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller.c->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller.c->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, switch_controller_empty)
{
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    resource_manager_, executor_,
    "test_controller_manager");
  std::string controller_type = test_controller::TEST_CONTROLLER_TYPE;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm->get_loaded_controllers().size());

  const auto UNSPECIFIED = 0;

  std::vector<std::string> start_controllers = {};
  std::vector<std::string> stop_controllers = {};

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0))
  ) << "Switch with no controllers specified";
  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      BEST_EFFORT, true, rclcpp::Duration(0, 0))
  ) << "Switch with no controllers specified";

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      UNSPECIFIED, true, rclcpp::Duration(0, 0))
  ) << "Switch with no controllers specified, unspecified strictness defaults to BEST_EFFORT";


  start_controllers = {"nonexistent_controller"};
  stop_controllers = {};
  EXPECT_EQ(
    controller_interface::return_type::ERROR,
    cm->switch_controller(
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0))
  ) << "STRICT switch with nonexistent controller specified";

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      BEST_EFFORT, true, rclcpp::Duration(0, 0))
  ) << "BEST_EFFORT switch with nonexistent controller specified";

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      UNSPECIFIED, true, rclcpp::Duration(0, 0))
  ) << "Unspecified switch with nonexistent controller specified, defaults to BEST_EFFORT";

  // From now on will only test STRICT and BEST_EFFORT

  start_controllers = {};
  stop_controllers = {"nonexistent_controller"};
  EXPECT_EQ(
    controller_interface::return_type::ERROR,
    cm->switch_controller(
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0))
  ) << "STRICT switch with nonexistent controller specified";

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      BEST_EFFORT, true, rclcpp::Duration(0, 0))
  ) << "BEST_EFFORT switch with nonexistent controller specified";


  start_controllers = {"nonexistent_controller"};
  stop_controllers = {"nonexistent_controller"};
  EXPECT_EQ(
    controller_interface::return_type::ERROR,
    cm->switch_controller(
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0))
  ) << "STRICT switch with nonexistent controller specified";

  EXPECT_EQ(
    controller_interface::return_type::SUCCESS,
    cm->switch_controller(
      start_controllers, stop_controllers,
      BEST_EFFORT, true, rclcpp::Duration(0, 0))
  ) << "BEST_EFFORT switch with nonexistent controller specified";
}

TEST_F(TestControllerManager, switch_controller)
{
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    resource_manager_, executor_,
    "test_controller_manager");
  std::string controller_type = test_controller::TEST_CONTROLLER_TYPE;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm->get_loaded_controllers()[0];

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());


  {  //  Test stopping an stopped controller
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1};

    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with stopped controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "BEST_EFFORT switch stopped controller specified";
  }


  { //  STRICT Combination of valid controller + invalid controller
    std::vector<std::string> start_controllers = {controller_name1, "nonexistent_controller"};
    std::vector<std::string> stop_controllers = {};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";

    start_controllers = {controller_name1};
    stop_controllers = {"nonexistent_controller"};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";
  }

  // Only testing with STRICT now for simplicity
  { //  Test starting an stopped controller, and stopping afterwards
    RCLCPP_INFO(
      cm->get_logger(),
      "Starting stopped controller");
    std::vector<std::string> start_controllers = {controller_name1};
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
    cm->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());


    // Stop controller
    start_controllers = {};
    stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm->get_logger(),
      "Stopping started controller");
    switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());
  }
}

TEST_F(TestControllerManager, switch_multiple_controllers)
{
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    resource_manager_, executor_,
    "test_controller_manager");
  std::string controller_type = test_controller::TEST_CONTROLLER_TYPE;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm->load_controller(controller_name1, controller_type));
  ASSERT_NO_THROW(cm->load_controller(controller_name2, controller_type));
  EXPECT_EQ(2u, cm->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm->get_loaded_controllers()[0];
  controller_manager::ControllerSpec abstract_test_controller2 =
    cm->get_loaded_controllers()[1];

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2.c->get_lifecycle_node()->get_current_state().id());

  // Only testing with STRICT now for simplicity
  { //  Test starting an stopped controller, and stopping afterwards
    RCLCPP_INFO(
      cm->get_logger(),
      "Starting stopped controller #1");
    std::vector<std::string> start_controllers = {controller_name1};
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
    cm->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_node()->get_current_state().id());

    // Stop controller 1, start controller 2
    start_controllers = {controller_name2};
    stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm->get_logger(),
      "Stopping controller #1, starting controller #2");
    switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_node()->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller2.c->get_lifecycle_node()->get_current_state().id());

    start_controllers = {};
    stop_controllers = {controller_name2};
    RCLCPP_INFO(
      cm->get_logger(),
      "Stopping controller #1, starting controller #2");
    switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );


    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_node()->get_current_state().id());
  }
}
