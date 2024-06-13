// Copyright 2024 Tokyo Robotics Inc.
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

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller_with_command/test_controller_with_command.hpp"

using test_controller_with_command::SIMULATE_COMMAND_ACTIVATE_VALUE;
using test_controller_with_command::SIMULATE_COMMAND_DEACTIVATE_VALUE;
using test_controller_with_command::TEST_CONTROLLER_CLASS_NAME;
using test_controller_with_command::TestControllerWithCommand;
const auto CONTROLLER_NAME = "test_controller1";
using strvec = std::vector<std::string>;

namespace
{
struct TestRestControllerStrictness
{
  int strictness = STRICT;
  std::future_status expected_future_status;
  controller_interface::return_type expected_return;
};
TestRestControllerStrictness test_strict{
  STRICT, std::future_status::ready, controller_interface::return_type::ERROR};
TestRestControllerStrictness test_best_effort{
  BEST_EFFORT, std::future_status::timeout, controller_interface::return_type::OK};
}  // namespace

class TestRestartController
: public ControllerManagerFixture<controller_manager::ControllerManager>,
  public testing::WithParamInterface<TestRestControllerStrictness>
{
public:
  void SetUp() override
  {
    SetupController();
    run_updater_ = false;
  }

  void SetupController()
  {
    // load controller
    test_controller = std::make_shared<TestControllerWithCommand>();
    cm_->add_controller(test_controller, CONTROLLER_NAME, TEST_CONTROLLER_CLASS_NAME);

    EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
    EXPECT_EQ(2u, test_controller.use_count());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  }

  void configure_and_check_test_controller()
  {
    // configure controller
    cm_->configure_controller(CONTROLLER_NAME);
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());

    // validate initial states
    EXPECT_EQ(0u, test_controller->activate_calls);
    EXPECT_EQ(0u, test_controller->deactivate_calls);
    EXPECT_TRUE(std::isnan(test_controller->simulate_command));
  }

  void start_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    switch_test_controllers(
      strvec{CONTROLLER_NAME}, strvec{}, strictness, expected_future_status,
      expected_interface_status);
  }

  void start_and_check_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    // Start controller
    start_test_controller(strictness, expected_future_status, expected_interface_status);

    // State should be active
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());

    // Only activate counter should be incremented
    ASSERT_EQ(1u, test_controller->activate_calls);
    ASSERT_EQ(0u, test_controller->deactivate_calls);

    // Controller command should be restart to ACTIVATE_VALUE
    ASSERT_EQ(SIMULATE_COMMAND_ACTIVATE_VALUE, test_controller->simulate_command);
  }

  void stop_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    switch_test_controllers(
      strvec{}, strvec{CONTROLLER_NAME}, strictness, expected_future_status,
      expected_interface_status);
  }

  void restart_test_controller(
    const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_interface_status =
      controller_interface::return_type::OK)
  {
    switch_test_controllers(
      strvec{CONTROLLER_NAME}, strvec{CONTROLLER_NAME}, strictness, expected_future_status,
      expected_interface_status);
  }

  std::shared_ptr<TestControllerWithCommand> test_controller;
};

TEST_P(TestRestartController, starting_and_stopping_a_controller)
{
  const auto test_param = GetParam();

  // Configure controller
  configure_and_check_test_controller();

  {
    // Start controller
    start_and_check_test_controller(test_param.strictness);
  }

  {
    // Stop controller
    stop_test_controller(test_param.strictness);

    // State should be inactive
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());

    // Only deactivate counter should be incremented
    ASSERT_EQ(1u, test_controller->activate_calls);
    ASSERT_EQ(1u, test_controller->deactivate_calls);

    // Controller command should be restart to DEACTIVATE_VALUE
    ASSERT_EQ(SIMULATE_COMMAND_DEACTIVATE_VALUE, test_controller->simulate_command);
  }
}

TEST_P(TestRestartController, can_restart_active_controller)
{
  const auto test_param = GetParam();

  // Configure controller
  configure_and_check_test_controller();

  {
    // Start controller before restart
    start_and_check_test_controller(test_param.strictness);
    const double OVERRIDE_COMMAND_VALUE = 12121212.0;

    // Override command to check restart behavior
    test_controller->simulate_command = OVERRIDE_COMMAND_VALUE;
    ASSERT_EQ(OVERRIDE_COMMAND_VALUE, test_controller->simulate_command);
  }

  {
    // Restart controller
    restart_test_controller(test_param.strictness);

    // State should be return active immediately (active -> inactive -> active)
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());

    // Both activate counter and deactivate counter are incremented
    ASSERT_EQ(2u, test_controller->activate_calls);
    ASSERT_EQ(1u, test_controller->deactivate_calls);

    // Controller command should be restart to ACTIVATE_VALUE
    ASSERT_EQ(SIMULATE_COMMAND_ACTIVATE_VALUE, test_controller->simulate_command);
  }
}

TEST_P(TestRestartController, restart_inactive_controller)
{
  const auto test_param = GetParam();

  // Configure controller
  configure_and_check_test_controller();

  // Restart controller without starting it
  restart_test_controller(
    test_param.strictness, test_param.expected_future_status, test_param.expected_return);

  // STRICT: can not restart inactive controller
  if (test_param.strictness == STRICT)
  {
    // State should not be changed
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());

    // Both activate counter and deactivate counter should not be incremented
    ASSERT_EQ(0u, test_controller->activate_calls);
    ASSERT_EQ(0u, test_controller->deactivate_calls);
  }

  // BEST_EFFORT: If restart is executed while inactive, only the start_controller process will be
  // effective, resulting in activation
  if (test_param.strictness == BEST_EFFORT)
  {
    // State changed to active
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());

    // Only activate counter and deactivate counter should not be incremented
    ASSERT_EQ(1u, test_controller->activate_calls);
    ASSERT_EQ(0u, test_controller->deactivate_calls);

    // Controller command should be restart to ACTIVATE_VALUE
    ASSERT_EQ(SIMULATE_COMMAND_ACTIVATE_VALUE, test_controller->simulate_command);
  }
}

TEST_P(TestRestartController, can_not_restart_unconfigured_controller)
{
  const auto test_param = GetParam();

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());

  // Can not restart unconfigured controller
  restart_test_controller(
    test_param.strictness, std::future_status::ready, test_param.expected_return);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
}

TEST_P(TestRestartController, can_not_restart_finalized_controller)
{
  const auto test_param = GetParam();

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());

  // Shutdown controller on purpose for testing
  ASSERT_EQ(
    test_controller->get_node()->shutdown().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  // Can not restart finalized controller
  restart_test_controller(
    test_param.strictness, std::future_status::ready, test_param.expected_return);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, test_controller->get_state().id());
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestRestartController, testing::Values(test_strict, test_best_effort));
