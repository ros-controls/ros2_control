// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_controller_interface.hpp"

#include <gmock/gmock.h>
#include <memory>

#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

template <class T, size_t N>
constexpr size_t arrlen(T (&)[N])
{
  return N;
}

TEST(TestableControllerInterface, check_init)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableControllerInterface controller;

  // try to get node when not initialized
  ASSERT_THROW(controller.get_node(), std::runtime_error);

  // initialize, create node
  ASSERT_EQ(
    controller.init("testable_controller_interface"), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  // update_rate is set to default 0
  ASSERT_EQ(controller.get_update_rate(), 0u);

  // Even after configure is 0
  controller.configure();
  ASSERT_EQ(controller.get_update_rate(), 0u);

  rclcpp::shutdown();
}

TEST(TestableControllerInterface, check_setting_update_rate_in_configure)
{
  // mocks the declaration of overrides parameters in a yaml file
  char const * const argv[] = {"", "--ros-args", "-p", "update_rate:=2812"};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableControllerInterface controller;
  // initialize, create node
  ASSERT_EQ(
    controller.init("testable_controller_interface"), controller_interface::return_type::OK);

  // initialize executor to be able to get parameter update
  auto executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);

  executor->add_node(controller.get_node()->get_node_base_interface());
  auto update_executor_spin_future =
    std::async(std::launch::async, [&]() -> void { executor->spin(); });
  // This sleep is needed to prevent a too fast test from ending before the
  // executor has began to spin, which causes it to hang
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(50ms);

  // update_rate is set to default 0 because it is set on configure
  ASSERT_EQ(controller.get_update_rate(), 0u);

  // Even after configure is 0
  controller.configure();
  ASSERT_EQ(controller.get_update_rate(), 2812u);

  // Test updating of update_rate parameter
  EXPECT_EQ(std::system("ros2 param set /testable_controller_interface update_rate 623"), 0);
  ASSERT_EQ(controller.get_update_rate(), 2812u);
  controller.get_node()->activate();
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.1));
  controller.get_node()->deactivate();
  controller.get_node()->cleanup();
  ASSERT_EQ(controller.get_update_rate(), 2812u);
  // It is first changed after controller is configured again.
  controller.configure();
  ASSERT_EQ(controller.get_update_rate(), 623u);

  executor->cancel();
  rclcpp::shutdown();
}

TEST(TestableControllerInterfaceInitError, check_init_with_failure)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableControllerInterfaceInitError controller;

  // initialize, create node
  ASSERT_EQ(
    controller.init("testable_controller_interface"), controller_interface::return_type::ERROR);

  rclcpp::shutdown();
}

TEST(TestableControllerInterfaceInitFailure, check_init_with_failure)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableControllerInterfaceInitFailure controller;

  // initialize, create node
  ASSERT_EQ(
    controller.init("testable_controller_interface"), controller_interface::return_type::ERROR);

  rclcpp::shutdown();
}
