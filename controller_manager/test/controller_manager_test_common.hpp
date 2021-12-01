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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "rclcpp/utilities.hpp"

#include "ros2_control_test_assets/descriptions.hpp"
#include "test_controller/test_controller.hpp"
#include "test_controller_failed_init/test_controller_failed_init.hpp"

constexpr auto STRICT = controller_manager_msgs::srv::SwitchController::Request::STRICT;
constexpr auto BEST_EFFORT = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

class ControllerManagerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        ros2_control_test_assets::minimal_robot_urdf),
      executor_, "test_controller_manager");
    run_updater_ = false;
  }

  void TearDown() { stopCmUpdater(); }

  void startCmUpdater()
  {
    run_updater_ = true;
    updater_ = std::thread([&](void) -> void {
      while (run_updater_)
      {
        cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  void stopCmUpdater()
  {
    if (run_updater_)
    {
      run_updater_ = false;
      updater_.join();
    }
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;

  std::thread updater_;
  bool run_updater_;
};

class ControllerManagerRunner
{
public:
  explicit ControllerManagerRunner(ControllerManagerFixture * cmf) : cmf_(cmf)
  {
    cmf_->startCmUpdater();
  }

  ~ControllerManagerRunner() { cmf_->stopCmUpdater(); }

  ControllerManagerFixture * cmf_;
};

class ControllerMock : public controller_interface::ControllerInterface
{
public:
  MOCK_METHOD0(update, controller_interface::return_type(void));
};

#endif  // CONTROLLER_MANAGER_TEST_COMMON_HPP_
