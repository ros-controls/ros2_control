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
  std::shared_ptr<rclcpp::executor::Executor> executor;
};

TEST_F(TestControllerManager, load_unknown_controller) {
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  ASSERT_THROW(
    cm.load_controller("unknown_package", "unknown_controller", "unknown_node"),
    std::runtime_error);
  ASSERT_THROW(
    cm.load_controller("controller_manager", "unknown_controller", "unknown_node"),
    std::runtime_error);
}

TEST_F(TestControllerManager, load_known_controller) {
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  std::string class_name = "test_controller::TestController";
  std::string package_name = "controller_manager";
  std::string controller_name = "test_controller";
  ASSERT_NO_THROW(cm.load_controller(package_name, class_name, controller_name));
  EXPECT_EQ(1u, cm.get_loaded_controller().size());

  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller =
    cm.get_loaded_controller()[0];

  auto lifecycle_node = abstract_test_controller->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, load2_known_controller) {
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  std::string class_name = "test_controller::TestController";
  std::string package_name = "controller_manager";

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm.load_controller(package_name, class_name, controller_name1));
  EXPECT_EQ(1u, cm.get_loaded_controller().size());
  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller1 =
    cm.get_loaded_controller()[0];
  EXPECT_STREQ(
    controller_name1.c_str(),
    abstract_test_controller1->get_lifecycle_node()->get_name());
  abstract_test_controller1->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1->get_lifecycle_node()->get_current_state().id());

  // load the same controller again with a different name
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm.load_controller(package_name, class_name, controller_name2));
  EXPECT_EQ(2u, cm.get_loaded_controller().size());
  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller2 =
    cm.get_loaded_controller()[1];
  EXPECT_STREQ(
    controller_name2.c_str(),
    abstract_test_controller2->get_lifecycle_node()->get_name());
  abstract_test_controller2->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, update) {
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  std::string class_name = "test_controller::TestController";
  std::string package_name = "controller_manager";
  std::string controller_name = "test_controller";
  ASSERT_NO_THROW(cm.load_controller(package_name, class_name, controller_name));

  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller =
    cm.get_loaded_controller()[0];

  auto lifecycle_node = abstract_test_controller->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller->get_lifecycle_node()->get_current_state().id());
}
