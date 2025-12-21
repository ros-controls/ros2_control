// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#include "test_controller_failed_init/test_controller_failed_init.hpp"

using test_controller::TEST_CONTROLLER_CLASS_NAME;
using ::testing::_;
using ::testing::Return;
const auto CONTROLLER_NAME_1 = "test_controller1";
using strvec = std::vector<std::string>;

class TestCleanupController : public ControllerManagerFixture<controller_manager::ControllerManager>
{
};

TEST_F(TestCleanupController, cleanup_unknown_controller)
{
  ASSERT_EQ(
    cm_->cleanup_controller("unknown_controller_name"), controller_interface::return_type::ERROR);
}

TEST_F(TestCleanupController, cleanup_controller_failed_init)
{
  auto controller_if = cm_->load_controller(
    "test_controller_failed_init",
    test_controller_failed_init::TEST_CONTROLLER_FAILED_INIT_CLASS_NAME);

  ASSERT_EQ(
    cm_->cleanup_controller("test_controller_failed_init"),
    controller_interface::return_type::ERROR);
}

TEST_F(TestCleanupController, cleanup_non_loaded_controller_fails)
{
  // try cleanup non-loaded controller
  EXPECT_EQ(cm_->cleanup_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
}

TEST_F(TestCleanupController, cleanup_unconfigured_controller)
{
  auto controller_if =
    cm_->load_controller(CONTROLLER_NAME_1, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_lifecycle_state().id());
  ASSERT_EQ(cm_->cleanup_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_lifecycle_state().id());
}

TEST_F(TestCleanupController, cleanup_inactive_controller)
{
  auto controller_if =
    cm_->load_controller(CONTROLLER_NAME_1, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_lifecycle_state().id());

  cm_->configure_controller(CONTROLLER_NAME_1);

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_lifecycle_state().id());
  ASSERT_EQ(cm_->cleanup_controller(CONTROLLER_NAME_1), controller_interface::return_type::OK);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_lifecycle_state().id());
}

TEST_F(TestCleanupController, cleanup_active_controller)
{
  auto controller_if =
    cm_->load_controller(CONTROLLER_NAME_1, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_NE(controller_if, nullptr);
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    controller_if->get_lifecycle_state().id());

  cm_->configure_controller(CONTROLLER_NAME_1);

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_if->get_lifecycle_state().id());

  switch_test_controllers(strvec{CONTROLLER_NAME_1}, strvec{}, STRICT);

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_lifecycle_state().id());

  ASSERT_EQ(cm_->cleanup_controller(CONTROLLER_NAME_1), controller_interface::return_type::ERROR);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_if->get_lifecycle_state().id());
}
