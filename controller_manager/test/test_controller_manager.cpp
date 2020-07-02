// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "controller_manager/controller_manager.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/utilities.hpp"

#include "test_robot_hardware/test_robot_hardware.hpp"

#include "./test_controller/test_controller.hpp"

class TestControllerManager : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
    robot->init();

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  std::shared_ptr<test_robot_hardware::TestRobotHardware> robot;
  std::shared_ptr<rclcpp::Executor> executor;
};

TEST_F(TestControllerManager, controller_lifecycle) {
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");

  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm.add_controller(test_controller, "test_controller");
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm.update());
  EXPECT_EQ(1u, test_controller->internal_counter);

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm.configure());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm.activate());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm.deactivate());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm.cleanup());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_node()->get_current_state().id());
}
