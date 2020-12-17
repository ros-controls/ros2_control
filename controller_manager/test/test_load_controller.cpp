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
#include "controller_manager/controller_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using ::testing::_;
using ::testing::Return;

class TestLoadController : public ControllerManagerFixture
{};

TEST_F(TestLoadController, load_unknown_controller)
{
  ASSERT_THROW(
    cm_->load_controller("unknown_controller_name", "unknown_controller_type"), std::runtime_error);
}

TEST_F(TestLoadController, load_and_configure_one_known_controller)
{
  ASSERT_NO_THROW(
    cm_->load_controller(
      "test_controller_01",
      test_controller::TEST_CONTROLLER_CLASS_NAME));
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  controller_manager::ControllerSpec abstract_test_controller =
    cm_->get_loaded_controllers()[0];

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller.c->get_current_state().id());

  cm_->configure_controller("test_controller_01");
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller.c->get_current_state().id());
}

TEST_F(TestLoadController, load_and_configure_two_known_controllers)
{
  std::string controller_type = test_controller::TEST_CONTROLLER_CLASS_NAME;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm_->get_loaded_controllers()[0];
  EXPECT_STREQ(
    controller_name1.c_str(), abstract_test_controller1.c->get_node()->get_name());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller1.c->get_current_state().id());

  // load the same controller again with a different name
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm_->load_controller(controller_name2, controller_type));
  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller2 =
    cm_->get_loaded_controllers()[1];
  EXPECT_STREQ(
    controller_name2.c_str(), abstract_test_controller2.c->get_node()->get_name());
  EXPECT_STREQ(
    controller_name2.c_str(), abstract_test_controller2.info.name.c_str());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller2.c->get_current_state().id());

  // Configure controllers
  cm_->configure_controller("test_controller1");
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_current_state().id());

  cm_->configure_controller("test_controller2");
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2.c->get_current_state().id());
}

TEST_F(TestLoadController, configure_controller)
{
  std::string controller_type = test_controller::TEST_CONTROLLER_CLASS_NAME;
  std::string controller_name1 = "test_controller1";

  // try configure not existing controller
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);

  // load the controller with name1
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm_->get_loaded_controllers()[0];

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller1.c->get_current_state().id());

  EXPECT_EQ(
    cm_->configure_controller(controller_name1), controller_interface::return_type::SUCCESS);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_current_state().id());

  { //  Start controller
    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting stopped controller");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};

    // First activation not possible because controller not configured
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_current_state().id());
  }
  // Can not configure active controller
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    abstract_test_controller1.c->get_current_state().id());

  {  // Stop controller
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping started controller");
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_current_state().id());
  }

  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(abstract_test_controller1.c);
  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  // Configure from inactive state: controller can no be cleaned-up
  test_controller->simulate_cleanup_failure = true;
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_current_state().id());
  EXPECT_EQ(0u, cleanup_calls);

  // Configure from inactive state
  test_controller->simulate_cleanup_failure = false;
  EXPECT_EQ(
    cm_->configure_controller(controller_name1), controller_interface::return_type::SUCCESS);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_current_state().id());
  EXPECT_EQ(1u, cleanup_calls);
}

TEST_F(TestLoadController, switch_controller_empty)
{
  std::string controller_type = test_controller::TEST_CONTROLLER_CLASS_NAME;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  const auto UNSPECIFIED = 0;

  { // test switch strictness
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {};

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "Switch with no controllers specified";
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "Switch with no controllers specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        UNSPECIFIED, true, rclcpp::Duration(0, 0))
    ) << "Switch with no controllers specified, unspecified strictness defaults to BEST_EFFORT";

    start_controllers = {"nonexistent_controller"};
    stop_controllers = {};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "BEST_EFFORT switch with nonexistent controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        UNSPECIFIED, true, rclcpp::Duration(0, 0))
    ) << "Unspecified switch with nonexistent controller specified, defaults to BEST_EFFORT";
  }

  { // From now on will only test STRICT and BEST_EFFORT
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {"nonexistent_controller"};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "BEST_EFFORT switch with nonexistent controller specified";

    start_controllers = {"nonexistent_controller"};
    stop_controllers = {"nonexistent_controller"};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "BEST_EFFORT switch with nonexistent controller specified";
  }
}

TEST_F(TestLoadController, switch_controller)
{
  std::string controller_type = test_controller::TEST_CONTROLLER_CLASS_NAME;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm_->get_loaded_controllers()[0];

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller1.c->get_current_state().id());

  {  //  Test stopping an stopped controller
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1};

    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with stopped controller specified";

    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        BEST_EFFORT, true, rclcpp::Duration(0, 0))
    ) << "BEST_EFFORT switch stopped controller specified";
  }

  { //  STRICT Combination of valid controller + invalid controller
    std::vector<std::string> start_controllers = {controller_name1, "nonexistent_controller"};
    std::vector<std::string> stop_controllers = {};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";

    start_controllers = {controller_name1};
    stop_controllers = {"nonexistent_controller"};
    EXPECT_EQ(
      controller_interface::return_type::ERROR,
      cm_->switch_controller(
        start_controllers, stop_controllers,
        STRICT, true, rclcpp::Duration(0, 0))
    ) << "STRICT switch with nonexistent controller specified";
  }

  // Only testing with STRICT now for simplicity
  { //  Test starting an stopped controller, and stopping afterwards
    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting stopped controller");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};

    // First activation not possible because controller not configured
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      abstract_test_controller1.c->get_current_state().id());

    // Activate configured controller
    cm_->configure_controller(controller_name1);
    switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_current_state().id());
  }

  { // Stop controller
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping started controller");
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_current_state().id());
  }
}

TEST_F(TestLoadController, switch_multiple_controllers)
{
  std::string controller_type = test_controller::TEST_CONTROLLER_CLASS_NAME;

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  ASSERT_NO_THROW(cm_->load_controller(controller_name2, controller_type));
  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 =
    cm_->get_loaded_controllers()[0];
  controller_manager::ControllerSpec abstract_test_controller2 =
    cm_->get_loaded_controllers()[1];

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller1.c->get_current_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    abstract_test_controller2.c->get_current_state().id());

  // Only testing with STRICT now for simplicity
  { //  Test starting an stopped controller, and stopping afterwards
    // configure controller 1
    cm_->configure_controller(controller_name1);

    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting stopped controller #1");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      abstract_test_controller2.c->get_current_state().id());
  }

  { // Stop controller 1, start controller 2
    // Fails starting of contorller 2 because it is not configured
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping controller #1, starting controller #2 fails");
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      abstract_test_controller2.c->get_current_state().id());

    // configure controller 2
    cm_->configure_controller(controller_name2);
  }

  { // Start controller 1 again
    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting stopped controller #1");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_current_state().id());
  }

  { // Stop controller 1, start controller 2
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {controller_name1};
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping controller #1, starting controller #2");
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller2.c->get_current_state().id());
  }

  { // stop controller 2
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name2};
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping controller #1, starting controller #2");
    auto switch_future = std::async(
      std::launch::async,
      &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers,
      STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(
      std::future_status::timeout,
      switch_future.wait_for(std::chrono::milliseconds(100))) <<
      "switch_controller should be blocking until next update cycle";
    cm_->update();
    EXPECT_EQ(
      controller_interface::return_type::SUCCESS,
      switch_future.get()
    );

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_current_state().id());
  }
}
