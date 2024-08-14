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

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

#include "std_msgs/msg/string.hpp"

#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
const auto STRICT = controller_manager_msgs::srv::SwitchController::Request::STRICT;
const auto BEST_EFFORT = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
const auto TEST_CM_NAME = "test_controller_manager";
}  // namespace
// Strictness structure for parameterized tests - shared between different tests
struct Strictness
{
  int strictness = STRICT;
  controller_interface::return_type expected_return;
  unsigned int expected_counter;
};
Strictness strict{STRICT, controller_interface::return_type::ERROR, 0u};
Strictness best_effort{BEST_EFFORT, controller_interface::return_type::OK, 1u};

// Forward definition to avid compile error - defined at the end of the file
template <typename CtrlMgr>
class ControllerManagerRunner;

template <typename CtrlMgr>
class ControllerManagerFixture : public ::testing::Test
{
public:
  explicit ControllerManagerFixture(
    const std::string & robot_description = ros2_control_test_assets::minimal_robot_urdf,
    const std::string & cm_namespace = "")
  : robot_description_(robot_description)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<CtrlMgr>(
      std::make_unique<hardware_interface::ResourceManager>(
        rm_node_->get_node_clock_interface(), rm_node_->get_node_logging_interface()),
      executor_, TEST_CM_NAME, cm_namespace);
    // We want to be able to not pass robot description immediately
    if (!robot_description_.empty())
    {
      pass_robot_description_to_cm_and_rm(robot_description_);
    }
    time_ = rclcpp::Time(0, 0, cm_->get_node_clock_interface()->get_clock()->get_clock_type());
  }

  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override { run_updater_ = false; }

  void TearDown() override { stopCmUpdater(); }

  void startCmUpdater()
  {
    run_updater_ = true;
    updater_ = std::thread(
      [&](void) -> void
      {
        while (run_updater_)
        {
          cm_->update(time_, rclcpp::Duration::from_seconds(0.01));
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

  void pass_robot_description_to_cm_and_rm(
    const std::string & robot_description = ros2_control_test_assets::minimal_robot_urdf)
  {
    // TODO(Manuel) : passing via topic not working in test setup, tested cm does
    // not receive msg. Have to check this...
    // this is just a workaround to skip passing - mimic topic call
    auto msg = std_msgs::msg::String();
    msg.data = robot_description;
    cm_->robot_description_callback(msg);
  }

  void switch_test_controllers(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers, const int strictness,
    const std::future_status expected_future_status = std::future_status::timeout,
    const controller_interface::return_type expected_return = controller_interface::return_type::OK)
  {
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, strictness, true, rclcpp::Duration(0, 0));

    ASSERT_EQ(expected_future_status, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner<CtrlMgr> cm_runner(this);
    EXPECT_EQ(expected_return, switch_future.get());
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<CtrlMgr> cm_;

  std::thread updater_;
  bool run_updater_;
  const std::string robot_description_;
  rclcpp::Time time_;

protected:
  rclcpp::Node::SharedPtr rm_node_ = std::make_shared<rclcpp::Node>("ResourceManager");
};

class TestControllerManagerSrvs
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  TestControllerManagerSrvs() {}

  void SetUp() override
  {
    ControllerManagerFixture::SetUp();
    SetUpSrvsCMExecutor();
  }

  void SetUpSrvsCMExecutor()
  {
    update_timer_ = cm_->create_wall_timer(
      std::chrono::milliseconds(10),
      [&]()
      {
        cm_->read(time_, PERIOD);
        cm_->update(time_, PERIOD);
        cm_->write(time_, PERIOD);
      });

    executor_->add_node(cm_);

    executor_spin_future_ = std::async(std::launch::async, [this]() -> void { executor_->spin(); });
    // This sleep is needed to prevent a too fast test from ending before the
    // executor has begun to spin, which causes it to hang
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // FIXME: This can be deleted!
  void TearDown() override { executor_->cancel(); }

  template <typename T>
  std::shared_ptr<typename T::Response> call_service_and_wait(
    rclcpp::Client<T> & client, std::shared_ptr<typename T::Request> request,
    rclcpp::Executor & service_executor, bool update_controller_while_spinning = false)
  {
    EXPECT_TRUE(client.wait_for_service(std::chrono::milliseconds(500)));
    auto result = client.async_send_request(request);
    // Wait for the result.
    if (update_controller_while_spinning)
    {
      while (service_executor.spin_until_future_complete(result, std::chrono::milliseconds(50)) !=
             rclcpp::FutureReturnCode::SUCCESS)
      {
        cm_->update(time_, rclcpp::Duration::from_seconds(0.01));
      }
    }
    else
    {
      EXPECT_EQ(
        service_executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);
    }
    return result.get();
  }

protected:
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::future<void> executor_spin_future_;
};

template <typename CtrlMgr>
class ControllerManagerRunner
{
public:
  explicit ControllerManagerRunner(ControllerManagerFixture<CtrlMgr> * cmf) : cmf_(cmf)
  {
    cmf_->startCmUpdater();
  }

  ~ControllerManagerRunner() { cmf_->stopCmUpdater(); }

  ControllerManagerFixture<CtrlMgr> * cmf_;
};

class ControllerMock : public controller_interface::ControllerInterface
{
public:
  MOCK_METHOD0(update, controller_interface::return_type(void));
};

#endif  // CONTROLLER_MANAGER_TEST_COMMON_HPP_
