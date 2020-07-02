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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/controller_manager.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/utilities.hpp"

#include "test_controller/test_controller.hpp"

#include "test_robot_hardware/test_robot_hardware.hpp"

using ::testing::_;
using ::testing::Return;

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

class ControllerMock : public controller_interface::ControllerInterface
{
public:
  MOCK_METHOD0(update, controller_interface::return_type(void));
};

class ControllerLoaderMock : public controller_manager::ControllerLoaderInterface
{
public:
  MOCK_METHOD1(create, controller_interface::ControllerInterfaceSharedPtr(const std::string &));
  MOCK_CONST_METHOD1(is_available, bool(const std::string &));
};

TEST_F(TestControllerManager, load_unknown_controller)
{
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  ASSERT_THROW(
    cm.load_controller("unknown_controller_name", "unknown_controller_type"), std::runtime_error);
}

TEST_F(TestControllerManager, load1_known_controller)
{
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  ASSERT_NO_THROW(cm.load_controller("test_controller_01", "test_controller"));
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());

  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, load2_known_controller)
{
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  std::string controller_type = "test_controller";

  // load the controller with name1
  std::string controller_name1 = "test_controller1";
  ASSERT_NO_THROW(cm.load_controller(controller_name1, controller_type));
  EXPECT_EQ(1u, cm.get_loaded_controllers().size());
  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller1 =
    cm.get_loaded_controllers()[0];
  EXPECT_STREQ(
    controller_name1.c_str(), abstract_test_controller1->get_lifecycle_node()->get_name());
  abstract_test_controller1->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1->get_lifecycle_node()->get_current_state().id());

  // load the same controller again with a different name
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm.load_controller(controller_name2, controller_type));
  EXPECT_EQ(2u, cm.get_loaded_controllers().size());
  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller2 =
    cm.get_loaded_controllers()[1];
  EXPECT_STREQ(
    controller_name2.c_str(), abstract_test_controller2->get_lifecycle_node()->get_name());
  abstract_test_controller2->get_lifecycle_node()->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, update)
{
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");
  ASSERT_NO_THROW(cm.load_controller("test_controller_01", "test_controller"));

  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller->get_lifecycle_node()->get_current_state().id());
}

TEST_F(TestControllerManager, register_controller_loader)
{
  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");

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

  std::shared_ptr<controller_interface::ControllerInterface> abstract_test_controller =
    cm.get_loaded_controllers()[0];

  auto lifecycle_node = abstract_test_controller->get_lifecycle_node();
  lifecycle_node->configure();
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller->get_lifecycle_node()->get_current_state().id());
}
