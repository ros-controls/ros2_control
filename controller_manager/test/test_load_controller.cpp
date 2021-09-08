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
#include <tuple>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using test_controller::TEST_CONTROLLER_CLASS_NAME;
using ::testing::_;
using ::testing::Return;
const auto controller_name1 = "test_controller1";
const auto controller_name2 = "test_controller2";
using strvec = std::vector<std::string>;

class TestLoadController : public ControllerManagerFixture
{
};

TEST_F(TestLoadController, load_unknown_controller)
{
  ASSERT_EQ(cm_->load_controller("unknown_controller_name", "unknown_controller_type"), nullptr);
}

TEST_F(TestLoadController, load_controller_failed_init)
{
  ASSERT_EQ(
    cm_->load_controller(
      "test_controller_failed_init",
      test_controller_failed_init::TEST_CONTROLLER_FAILED_INIT_CLASS_NAME),
    nullptr);
}

TEST_F(TestLoadController, load_and_configure_one_known_controller)
{
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_current_state().id());

  cm_->configure_controller(controller_name1);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
}

TEST_F(TestLoadController, load_and_configure_two_known_controllers)
{
  // load the controller with name1
  auto controller_if1 = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if1, nullptr);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_STREQ(controller_name1, controller_if1->get_node()->get_name());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if1->get_current_state().id());

  // load the same controller again with a different name
  auto controller_if2 = cm_->load_controller(controller_name2, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if2, nullptr);
  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  EXPECT_STREQ(controller_name2, controller_if2->get_node()->get_name());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if2->get_current_state().id());

  // Configure controllers
  cm_->configure_controller(controller_name1);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_current_state().id());

  cm_->configure_controller(controller_name2);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_current_state().id());
}

TEST_F(TestLoadController, configuring_non_loaded_controller_fails)
{
  // try configure non-loaded controller
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
}

TEST_F(TestLoadController, can_configure_loaded_controller)
{
  // load the controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_current_state().id());

  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
}

TEST_F(TestLoadController, can_start_configured_controller)
{
  // load and start controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);

  {  //  Start controller
    RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};

    // First activation not possible because controller not configured
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_current_state().id());
  }
}

TEST_F(TestLoadController, can_not_configure_active_controller)
{
  // load and start controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);

  //  Start controller
  strvec start_controllers = {controller_name1};
  strvec stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

  // Can not configure active controller
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_current_state().id());
}

TEST_F(TestLoadController, can_stop_active_controller)
{
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);
  // load and start controller with name1
  {
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      strvec{controller_name1}, strvec{}, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }
  // Stop controller
  RCLCPP_INFO(cm_->get_logger(), "Stopping started controller");
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_, strvec{},
    strvec{controller_name1}, STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
}

TEST_F(TestLoadController, can_not_start_finalized_controller)
{
  // load and start controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_current_state().id());

  // Shutdown controller on purpose for testing
  ASSERT_EQ(controller_if->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  //  Start controller
  strvec start_controllers = {controller_name1};
  strvec stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::ready, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::ERROR, switch_future.get());

  // Can not configure unconfigured controller
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, controller_if->get_current_state().id());
}

TEST_F(TestLoadController, inactive_controller_cannot_be_cleaned_up)
{
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);
  // load and start controller with name1
  {
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      strvec{controller_name1}, strvec{}, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }
  // Stop controller
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_, strvec{},
    strvec{controller_name1}, STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());

  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(controller_if);
  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  // Configure from inactive state: controller can no be cleaned-up
  test_controller->simulate_cleanup_failure = true;
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::ERROR);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
  EXPECT_EQ(0u, cleanup_calls);
}

TEST_F(TestLoadController, inactive_controller_cannot_be_configured)
{
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);

  // load and start controller with name1
  {
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      strvec{controller_name1}, strvec{}, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }
  // Stop controller
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_, strvec{},
    strvec{controller_name1}, STRICT, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());

  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(controller_if);
  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  // Configure from inactive state
  test_controller->simulate_cleanup_failure = false;
  EXPECT_EQ(cm_->configure_controller(controller_name1), controller_interface::return_type::OK);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
  EXPECT_EQ(1u, cleanup_calls);
}

class SwitchTest
: public TestLoadController,
  public ::testing::WithParamInterface<
    std::tuple<controller_interface::return_type, int, strvec, strvec, std::string>>
{
};

const auto UNSPECIFIED = 0;
const auto EMPTY_STR_VEC = strvec{};
const auto NONEXISTENT_CONTROLLER = strvec{"nonexistent_controller"};
const auto VALID_CONTROLLER = strvec{controller_name1};
const auto VALID_PLUS_NONEXISTENT_CONTROLLERS = strvec{controller_name1, "nonexistent_controller"};

TEST_P(SwitchTest, EmptyListOrNonExistentTest)
{
  // load the controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_current_state().id());

  auto params = GetParam();
  auto result = std::get<0>(params);
  auto strictness = std::get<1>(params);
  auto start_controllers = std::get<2>(params);
  auto stop_controllers = std::get<3>(params);
  auto error_message = std::get<4>(params);

  EXPECT_EQ(
    result, cm_->switch_controller(
              start_controllers, stop_controllers, strictness, true, rclcpp::Duration(0, 0)))
    << error_message;
}

// TODO(anyone): For Galactic+, follow up on https://github.com/ros-controls/ros2_control/issues/406
#ifdef __APPLE__
INSTANTIATE_TEST_CASE_P(
  EmptyListOrNonExistentTest, SwitchTest,
  ::testing::Values(
    // empty lists
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, STRICT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    // combination of empty and non-existent controller
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    // valid controller used
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_PLUS_NONEXISTENT_CONTROLLERS,
      EMPTY_STR_VEC, "Switch with valid and nonexistent controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_CONTROLLER, NONEXISTENT_CONTROLLER,
      "Switch with  valid and nonexistent controller specified"))
  // cppcheck-suppress syntaxError
  , );
#else
INSTANTIATE_TEST_CASE_P(
  EmptyListOrNonExistentTest, SwitchTest,
  ::testing::Values(
    // empty lists
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, STRICT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified"),
    // combination of empty and non-existent controller
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER, "Switch with nonexistent start and stop controllers specified"),
    // valid controller used
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller specified"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_PLUS_NONEXISTENT_CONTROLLERS,
      EMPTY_STR_VEC, "Switch with valid and nonexistent controller specified"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_CONTROLLER, NONEXISTENT_CONTROLLER,
      "Switch with  valid and nonexistent controller specified")));
#endif

TEST_F(TestLoadController, starting_and_stopping_a_controller)
{
  // load the controller with name1
  auto controller_if = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_current_state().id());

  // Only testing with STRICT now for simplicity
  {
    // Test starting unconfigured controller, and starting configured afterwards
    RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller");
    strvec start_controllers = {controller_name1};
    strvec stop_controllers = {};

    // First activation not possible because controller not configured
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::ready, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    {
      ControllerManagerRunner cm_runner(this);
      EXPECT_EQ(controller_interface::return_type::ERROR, switch_future.get());
    }

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      controller_if->get_current_state().id());

    // Activate configured controller
    cm_->configure_controller(controller_name1);
    switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    {
      ControllerManagerRunner cm_runner(this);
      EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    }
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_current_state().id());
  }

  {  // Stop controller
    strvec start_controllers = {};
    strvec stop_controllers = {controller_name1};
    RCLCPP_INFO(cm_->get_logger(), "Stopping started controller");
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_current_state().id());
  }
}

TEST_F(TestLoadController, switch_multiple_controllers)
{
  // load the controller with name1
  auto controller_if1 = cm_->load_controller(controller_name1, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if1, nullptr);
  auto controller_if2 = cm_->load_controller(controller_name2, TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if2, nullptr);
  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if1->get_current_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if2->get_current_state().id());

  // Only testing with STRICT now for simplicity
  {  //  Test starting an stopped controller, and stopping afterwards
    // configure controller 1
    cm_->configure_controller(controller_name1);

    RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller #1");
    strvec start_controllers = {controller_name1};
    strvec stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      controller_if2->get_current_state().id());
  }

  {  // Stop controller 1, start controller 2
    // Both fails because controller 2 because it is not configured and STRICT is used
    strvec start_controllers = {controller_name2};
    strvec stop_controllers = {controller_name1};
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1, starting controller #2 fails");
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::ready, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::ERROR, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      controller_if2->get_current_state().id());

    // configure controller 2
    cm_->configure_controller(controller_name2);
  }

  {  // Stop controller 1
    strvec start_controllers = {};
    strvec stop_controllers = {controller_name1};
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1, starting controller #2 fails");
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_current_state().id());
  }

  {  // Start controller 1 again
    RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller #1");
    strvec start_controllers = {controller_name1};
    strvec stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_current_state().id());
  }

  {  // Stop controller 1, start controller 2
    strvec start_controllers = {controller_name2};
    strvec stop_controllers = {controller_name1};
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1, starting controller #2");
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_current_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if2->get_current_state().id());
  }

  {  // stop controller 2
    strvec start_controllers = {};
    strvec stop_controllers = {controller_name2};
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1, starting controller #2");
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_current_state().id());
  }
}
