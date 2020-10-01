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

#ifndef CONTROLLER_MANAGER_TEST_COMMON_HPP_
#define CONTROLLER_MANAGER_TEST_COMMON_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "rclcpp/utilities.hpp"
#include "test_controller/test_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"


constexpr auto STRICT = controller_manager_msgs::srv::SwitchController::Request::STRICT;
constexpr auto BEST_EFFORT = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

class TestControllerManager : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
    robot_->init();

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  std::shared_ptr<test_robot_hardware::TestRobotHardware> robot_;
  std::shared_ptr<rclcpp::executor::Executor> executor_;
};

class ControllerMock : public controller_interface::ControllerInterface
{
public:
  MOCK_METHOD0(update, controller_interface::return_type(void));
};

constexpr char MOCK_TEST_CONTROLLER_NAME[] = "mock_test_controller";
constexpr char MOCK_TEST_CONTROLLER_TYPE[] = "ControllerMock";

class ControllerLoaderMock : public controller_manager::ControllerLoaderInterface
{
public:
  ControllerLoaderMock()
  : controller_manager::ControllerLoaderInterface("controller_interface::MockControllerInterface")
  {}
  MOCK_METHOD(
    controller_interface::ControllerInterfaceSharedPtr, create, (const std::string &),
    (override));
  MOCK_METHOD(
    controller_interface::ControllerInterfaceNewComponentsSharedPtr, create_new_components,
    (const std::string &), (override));
  MOCK_METHOD(bool, is_available, (const std::string &), (const, override));
  std::vector<std::string> get_declared_classes() const override
  {
    return {MOCK_TEST_CONTROLLER_NAME};
  }
  MOCK_METHOD(void, reload, (), (override));
};

#endif  // CONTROLLER_MANAGER_TEST_COMMON_HPP_
