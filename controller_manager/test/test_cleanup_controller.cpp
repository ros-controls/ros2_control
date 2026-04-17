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
#include "test_chainable_controller/test_chainable_controller.hpp"
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

// A chainable controller whose export_reference_interfaces throws.
class ThrowingChainableController : public test_chainable_controller::TestChainableController
{
public:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    throw std::runtime_error("Simulated export failure");
  }
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

TEST_F(TestCleanupController, configure_chainable_with_and_without_interfaces_should_not_hang)
{
  auto bad_controller = std::make_shared<test_chainable_controller::TestChainableController>();

  cm_->add_controller(
    bad_controller, "bad_chainable_controller",
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  auto ret = cm_->configure_controller("bad_chainable_controller");
  EXPECT_EQ(controller_interface::return_type::ERROR, ret);

  // After a failed configure, the controller should be rolled back to UNCONFIGURED.
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    bad_controller->get_lifecycle_state().id())
    << "Controller should be rolled back to UNCONFIGURED after configure failure, "
       "but is in state: "
    << bad_controller->get_lifecycle_state().label();
  
  // Add an interface and configure. Controller should reach INACTIVE state
  bad_controller->set_reference_interface_names({"joint1/position"});

  ret = cm_->configure_controller("bad_chainable_controller");
  EXPECT_EQ(controller_interface::return_type::OK, ret);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    bad_controller->get_lifecycle_state().id())
    << "Controller should reach INACTIVE after successful configure, "
       "but is in state: "
    << bad_controller->get_lifecycle_state().label();
}

TEST_F(TestCleanupController, configure_chainable_with_throwing_export_should_cleanup)
{
  auto throwing_controller = std::make_shared<ThrowingChainableController>();

  cm_->add_controller(
    throwing_controller, "throwing_chainable_controller",
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  auto ret = cm_->configure_controller("throwing_chainable_controller");
  EXPECT_EQ(controller_interface::return_type::ERROR, ret);

  // After an exception during export, the controller should be rolled back to UNCONFIGURED
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    throwing_controller->get_lifecycle_state().id())
    << "Controller should be rolled back to UNCONFIGURED after export exception, "
       "but is in state: "
    << throwing_controller->get_lifecycle_state().label();
}
