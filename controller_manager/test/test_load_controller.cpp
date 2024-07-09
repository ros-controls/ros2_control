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
#include "test_controller/test_controller.hpp"

using test_controller::TEST_CONTROLLER_CLASS_NAME;
using ::testing::_;
using ::testing::Return;
const auto CONTROLLER_NAME_1 = "test_controller1";
const auto CONTROLLER_NAME_2 = "test_controller2";
using strvec = std::vector<std::string>;

class TestLoadController : public ControllerManagerFixture<controller_manager::ControllerManager>
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

TEST_F(TestLoadController, configuring_non_loaded_controller_fails)
{
  // try configure non-loaded controller
  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
}

TEST_F(TestLoadController, can_set_and_get_non_default_update_rate)
{
  auto controller_if =
    cm_->load_controller("test_controller_01", test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

  controller_if->get_node()->set_parameter({"update_rate", 1337});

  cm_->configure_controller("test_controller_01");
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());

  EXPECT_EQ(1337u, controller_if->get_update_rate());
}

class TestLoadedController : public TestLoadController
{
public:
  controller_interface::ControllerInterfaceBaseSharedPtr controller_if{nullptr};

  void SetUp() override
  {
    TestLoadController::SetUp();

    controller_if = cm_->load_controller(CONTROLLER_NAME_1, TEST_CONTROLLER_CLASS_NAME);
    ASSERT_NE(controller_if, nullptr);
  }

  void start_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    switch_test_controllers(
      strvec{CONTROLLER_NAME_1}, strvec{}, strictness, expected_future_status,
      expected_interface_status);
  }

  void stop_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    switch_test_controllers(
      strvec{}, strvec{CONTROLLER_NAME_1}, strictness, expected_future_status,
      expected_interface_status);
  }
};

class TestLoadedControllerParametrized : public TestLoadedController,
                                         public testing::WithParamInterface<Strictness>
{
};

TEST_P(TestLoadedControllerParametrized, load_and_configure_one_known_controller)
{
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

  cm_->configure_controller(CONTROLLER_NAME_1);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());
}

TEST_P(TestLoadedControllerParametrized, can_start_configured_controller)
{
  const auto test_param = GetParam();

  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);
  start_test_controller(test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_state().id());
}

TEST_P(TestLoadedControllerParametrized, can_stop_active_controller)
{
  const auto test_param = GetParam();

  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);

  start_test_controller(test_param.strictness);

  // Stop controller
  stop_test_controller(test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());
}

TEST_P(TestLoadedControllerParametrized, starting_and_stopping_a_controller)
{
  const auto test_param = GetParam();

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

  {  // Test starting unconfigured controller, and starting configured afterwards
    start_test_controller(
      test_param.strictness, std::future_status::ready, test_param.expected_return);

    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

    // Activate configured controller
    {
      ControllerManagerRunner cm_runner(this);
      cm_->configure_controller(CONTROLLER_NAME_1);
    }
    start_test_controller(test_param.strictness);
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_state().id());
  }

  {  // Stop controller
    stop_test_controller(test_param.strictness);
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());
  }
}

TEST_P(TestLoadedControllerParametrized, can_not_configure_active_controller)
{
  const auto test_param = GetParam();

  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);
  start_test_controller(test_param.strictness);

  // Can not configure active controller
  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_state().id());
}

TEST_P(TestLoadedControllerParametrized, can_not_start_finalized_controller)
{
  const auto test_param = GetParam();

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

  // Shutdown controller on purpose for testing
  ASSERT_EQ(
    controller_if->get_node()->shutdown().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  //  Start controller
  start_test_controller(
    test_param.strictness, std::future_status::ready, test_param.expected_return);

  // Can not configure finalize controller
  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, controller_if->get_state().id());
}

TEST_P(TestLoadedControllerParametrized, inactive_controller_cannot_be_cleaned_up)
{
  const auto test_param = GetParam();

  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);

  start_test_controller(test_param.strictness);

  stop_test_controller(test_param.strictness);

  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());

  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(controller_if);
  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  // Configure from inactive state: controller can no be cleaned-up
  test_controller->simulate_cleanup_failure = true;
  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());
  EXPECT_EQ(0u, cleanup_calls);
}

TEST_P(TestLoadedControllerParametrized, inactive_controller_cannot_be_configured)
{
  const auto test_param = GetParam();

  EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);

  start_test_controller(test_param.strictness);

  stop_test_controller(test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());

  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(controller_if);
  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  // Configure from inactive state
  test_controller->simulate_cleanup_failure = false;
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(cm_->configure_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);
  }
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_state().id());
  EXPECT_EQ(1u, cleanup_calls);
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestLoadedControllerParametrized, testing::Values(strict, best_effort));

class SwitchTest
: public TestLoadedController,
  public ::testing::WithParamInterface<
    std::tuple<controller_interface::return_type, int, strvec, strvec, std::string>>
{
};

const auto UNSPECIFIED = 0;
const auto EMPTY_STR_VEC = strvec{};
const auto NONEXISTENT_CONTROLLER = strvec{"nonexistent_controller"};
const auto VALID_CONTROLLER = strvec{CONTROLLER_NAME_1};
const auto VALID_PLUS_NONEXISTENT_CONTROLLERS = strvec{CONTROLLER_NAME_1, "nonexistent_controller"};

TEST_P(SwitchTest, EmptyListOrNonExistentTest)
{
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if->get_state().id());

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

INSTANTIATE_TEST_SUITE_P(
  EmptyListOrNonExistentTest, SwitchTest,
  ::testing::Values(
    // empty lists
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified and strictness UNSPECIFIED didn't return OK"),
    std::make_tuple(
      controller_interface::return_type::OK, STRICT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified and strictness STRICT didn't return OK"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, EMPTY_STR_VEC,
      "Switch with no controllers specified and strictness BEST_EFFORT didn't return OK"),
    // combination of empty and non-existent controller
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified and strictness UNSPECIFIED didn't return "
      "OK"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified and strictness STRICT didn't return "
      "ERROR"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, EMPTY_STR_VEC,
      "Switch with nonexistent start controller specified and strictness BEST_EFFORT didn't return "
      "OK"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified and strictness UNSPECIFIED didn't return "
      "OK"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified and strictness STRICT didn't return "
      "ERROR"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, EMPTY_STR_VEC, NONEXISTENT_CONTROLLER,
      "Switch with nonexistent stop controller specified and strictness BEST_EFFORT didn't return "
      "OK"),
    std::make_tuple(
      controller_interface::return_type::OK, UNSPECIFIED, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER,
      "Switch with nonexistent start and stop controllers specified, and strictness UNSPECIFIED, "
      "didn't return OK"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER,
      "Switch with nonexistent start and stop controllers specified, and strictness STRICT, didn't "
      "return ERROR"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER,
      NONEXISTENT_CONTROLLER,
      "Switch with nonexistent start and stop controllers specified, and strictness BEST_EFFORT, "
      "didn't return OK"),
    // valid controller used
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller and nonexistent start controller specified, and "
      "strictness STRICT, didn't return ERROR"),
    std::make_tuple(
      controller_interface::return_type::OK, BEST_EFFORT, NONEXISTENT_CONTROLLER, VALID_CONTROLLER,
      "Switch with valid stopped controller specified, nonexistent start controller and strictness "
      "BEST_EFFORT didn't return OK"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_PLUS_NONEXISTENT_CONTROLLERS,
      EMPTY_STR_VEC,
      "Switch with valid and nonexistent start controller specified and strictness STRICT didn't "
      "return ERROR"),
    std::make_tuple(
      controller_interface::return_type::ERROR, STRICT, VALID_CONTROLLER, NONEXISTENT_CONTROLLER,
      "Switch with valid start controller and nonexistent controller specified, and strinctness "
      "STRICT, didn't return ERROR")));

class TestTwoLoadedControllers : public TestLoadController,
                                 public testing::WithParamInterface<Strictness>
{
public:
  controller_interface::ControllerInterfaceBaseSharedPtr controller_if1{nullptr};
  controller_interface::ControllerInterfaceBaseSharedPtr controller_if2{nullptr};

  void SetUp() override
  {
    TestLoadController::SetUp();
    controller_if1 = cm_->load_controller(CONTROLLER_NAME_1, TEST_CONTROLLER_CLASS_NAME);
    ASSERT_NE(controller_if1, nullptr);
    EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
    controller_if2 = cm_->load_controller(CONTROLLER_NAME_2, TEST_CONTROLLER_CLASS_NAME);
    ASSERT_NE(controller_if2, nullptr);
    EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if1->get_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if2->get_state().id());
  }
};

TEST_F(TestTwoLoadedControllers, load_and_configure_two_known_controllers)
{
  cm_->configure_controller(CONTROLLER_NAME_1);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_state().id());

  cm_->configure_controller(CONTROLLER_NAME_2);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_state().id());
}

TEST_P(TestTwoLoadedControllers, switch_multiple_controllers)
{
  const auto test_param = GetParam();

  cm_->configure_controller(CONTROLLER_NAME_1);

  // Start controller #1
  RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller #1");
  switch_test_controllers(strvec{CONTROLLER_NAME_1}, strvec{}, test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if2->get_state().id());

  // Stop controller 1, start controller 2
  // Both fail because controller 2 because it is not configured and STRICT is used
  RCLCPP_INFO(
    cm_->get_logger(), "Stopping controller #1, starting unconfigured controller #2 fails (%s)",
    (test_param.strictness == STRICT ? "STRICT" : "BEST EFFORT"));
  // TODO(destogl): fix this test for BEST_EFFORT - probably related to:
  // https://github.com/ros-controls/ros2_control/pull/582#issuecomment-1029931561
  //   switch_test_controllers(
  //     strvec{CONTROLLER_NAME_2}, strvec{CONTROLLER_NAME_1}, test_param.strictness,
  //     std::future_status::ready, controller_interface::return_type::ERROR);
  switch_test_controllers(
    strvec{CONTROLLER_NAME_2}, strvec{CONTROLLER_NAME_1}, STRICT, std::future_status::ready,
    controller_interface::return_type::ERROR);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, controller_if2->get_state().id());

  {
    ControllerManagerRunner cm_runner(this);
    cm_->configure_controller(CONTROLLER_NAME_2);
  }

  // Stop controller 1
  RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1");
  switch_test_controllers(strvec{}, strvec{CONTROLLER_NAME_1}, test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_state().id());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_state().id());

  // Start controller 1 again
  RCLCPP_INFO(cm_->get_logger(), "Starting stopped controller #1");
  switch_test_controllers(strvec{CONTROLLER_NAME_1}, strvec{}, test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if1->get_state().id());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_state().id());

  // Stop controller 1, start controller 2
  RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1, starting controller #2");
  switch_test_controllers(
    strvec{CONTROLLER_NAME_2}, strvec{CONTROLLER_NAME_1}, test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if1->get_state().id());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if2->get_state().id());

  // Stop controller 2
  RCLCPP_INFO(cm_->get_logger(), "Stopping controller #2");
  switch_test_controllers(strvec{}, strvec{CONTROLLER_NAME_2}, test_param.strictness);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if2->get_state().id());
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestTwoLoadedControllers, testing::Values(strict, best_effort));
