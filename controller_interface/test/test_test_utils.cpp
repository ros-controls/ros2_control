// Copyright 2026 AIT - Austrian Institute of Technology GmbH
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

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/test_utils.hpp"
#include "gmock/gmock.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace
{
using controller_interface::CallbackReturn;

class LifecycleTestController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn configure_return = CallbackReturn::SUCCESS;
  CallbackReturn activate_return = CallbackReturn::SUCCESS;
  CallbackReturn deactivate_return = CallbackReturn::SUCCESS;
  CallbackReturn cleanup_return = CallbackReturn::SUCCESS;
  CallbackReturn shutdown_return = CallbackReturn::SUCCESS;

  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override
  {
    return controller_interface::return_type::OK;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /* previous_state */) override
  {
    return configure_return;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /* previous_state */) override
  {
    return activate_return;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /* previous_state */) override
  {
    return deactivate_return;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /* previous_state */) override
  {
    return cleanup_return;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /* previous_state */) override
  {
    return shutdown_return;
  }
};

class MockLifecycleState
{
public:
  explicit MockLifecycleState(const uint8_t state_id) : state_id_(state_id) {}

  uint8_t id() const { return state_id_; }

private:
  uint8_t state_id_;
};

class MockLifecycleNode
{
public:
  MOCK_METHOD(MockLifecycleState, activate, (), ());
  MOCK_METHOD(MockLifecycleState, deactivate, (), ());
  MOCK_METHOD(MockLifecycleState, cleanup, (), ());
  MOCK_METHOD(MockLifecycleState, shutdown, (), ());
};

class MockController
{
public:
  MOCK_METHOD(MockLifecycleState, configure, (), ());
  MOCK_METHOD(std::shared_ptr<MockLifecycleNode>, get_node, (), ());
};

class ControllerInterfaceTestUtils : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<LifecycleTestController> make_controller()
  {
    static size_t controller_index = 0;
    auto controller = std::make_unique<LifecycleTestController>();

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_utils_controller_" + std::to_string(controller_index++);
    params.robot_description = "";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_namespace = "";
    params.node_options = controller->define_custom_node_options();

    const auto init_result = controller->init(params);
    EXPECT_EQ(init_result, controller_interface::return_type::OK);
    if (init_result != controller_interface::return_type::OK)
    {
      return nullptr;
    }
    return controller;
  }
};

// need to add this because cppcheck fails with "syntax error" on the TEST_F macros, even though
// they are valid and compile fine
// cppcheck-suppress syntaxError
TEST_F(ControllerInterfaceTestUtils, configure_succeeds_returns_true_for_inactive_state)
{
  auto controller = make_controller();
  EXPECT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
}

TEST_F(ControllerInterfaceTestUtils, configure_succeeds_returns_false_for_unconfigured_state)
{
  auto controller = make_controller();
  controller->configure_return = CallbackReturn::FAILURE;
  EXPECT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(false));
}

TEST_F(ControllerInterfaceTestUtils, configure_succeeds_throws_for_unexpected_state)
{
  auto controller = std::make_unique<MockController>();
  EXPECT_CALL(*controller, configure())
    .WillOnce(
      ::testing::Return(MockLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)));

  EXPECT_THROW(controller_interface::configure_succeeds(controller), std::runtime_error);
}

TEST_F(ControllerInterfaceTestUtils, activate_succeeds_returns_true_for_active_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));

  EXPECT_THAT(controller_interface::activate_succeeds(controller), ::testing::Eq(true));
}

TEST_F(ControllerInterfaceTestUtils, activate_succeeds_returns_false_for_inactive_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  controller->activate_return = CallbackReturn::FAILURE;

  EXPECT_THAT(controller_interface::activate_succeeds(controller), ::testing::Eq(false));
}

TEST_F(ControllerInterfaceTestUtils, activate_succeeds_throws_for_unexpected_state)
{
  auto node = std::make_shared<MockLifecycleNode>();
  auto controller = std::make_unique<MockController>();
  EXPECT_CALL(*controller, get_node()).WillOnce(::testing::Return(node));
  EXPECT_CALL(*node, activate())
    .WillOnce(
      ::testing::Return(
        MockLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)));

  EXPECT_THROW(controller_interface::activate_succeeds(controller), std::runtime_error);
}

TEST_F(ControllerInterfaceTestUtils, deactivate_succeeds_returns_true_for_inactive_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  ASSERT_THAT(controller_interface::activate_succeeds(controller), ::testing::Eq(true));

  EXPECT_THAT(controller_interface::deactivate_succeeds(controller), ::testing::Eq(true));
}

TEST_F(ControllerInterfaceTestUtils, deactivate_succeeds_returns_false_for_active_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  ASSERT_THAT(controller_interface::activate_succeeds(controller), ::testing::Eq(true));
  controller->deactivate_return = CallbackReturn::FAILURE;

  EXPECT_THAT(controller_interface::deactivate_succeeds(controller), ::testing::Eq(false));
}

TEST_F(ControllerInterfaceTestUtils, deactivate_succeeds_throws_for_unexpected_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  ASSERT_THAT(controller_interface::activate_succeeds(controller), ::testing::Eq(true));
  controller->deactivate_return = CallbackReturn::ERROR;

  EXPECT_THROW(controller_interface::deactivate_succeeds(controller), std::runtime_error);
}

TEST_F(ControllerInterfaceTestUtils, cleanup_succeeds_returns_true_for_unconfigured_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));

  EXPECT_THAT(controller_interface::cleanup_succeeds(controller), ::testing::Eq(true));
}

TEST_F(ControllerInterfaceTestUtils, cleanup_succeeds_returns_false_for_inactive_state)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  controller->cleanup_return = CallbackReturn::FAILURE;

  EXPECT_THAT(controller_interface::cleanup_succeeds(controller), ::testing::Eq(false));
}

TEST_F(ControllerInterfaceTestUtils, cleanup_succeeds_throws_for_unexpected_state)
{
  auto node = std::make_shared<MockLifecycleNode>();
  auto controller = std::make_unique<MockController>();
  EXPECT_CALL(*controller, get_node()).WillOnce(::testing::Return(node));
  EXPECT_CALL(*node, cleanup())
    .WillOnce(
      ::testing::Return(MockLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)));

  EXPECT_THROW(controller_interface::cleanup_succeeds(controller), std::runtime_error);
}

TEST_F(ControllerInterfaceTestUtils, shutdown_succeeds_returns_true_for_finalized_state)
{
  auto controller = make_controller();
  EXPECT_THAT(controller_interface::shutdown_succeeds(controller), ::testing::Eq(true));
}

TEST_F(ControllerInterfaceTestUtils, shutdown_succeeds_throws_for_real_controller_on_error)
{
  auto controller = make_controller();
  ASSERT_THAT(controller_interface::configure_succeeds(controller), ::testing::Eq(true));
  controller->shutdown_return = CallbackReturn::ERROR;

  EXPECT_THROW(controller_interface::shutdown_succeeds(controller), std::runtime_error);
}

TEST_F(ControllerInterfaceTestUtils, shutdown_succeeds_throws_for_unexpected_state)
{
  auto node = std::make_shared<MockLifecycleNode>();
  auto controller = std::make_unique<MockController>();
  EXPECT_CALL(*controller, get_node()).WillOnce(::testing::Return(node));
  EXPECT_CALL(*node, shutdown())
    .WillOnce(
      ::testing::Return(MockLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)));

  EXPECT_THROW(controller_interface::shutdown_succeeds(controller), std::runtime_error);
}

}  // namespace
