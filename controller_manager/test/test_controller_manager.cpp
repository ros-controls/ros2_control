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
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
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
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    robot, executor,
    "test_controller_manager");


  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm->add_controller(
    test_controller, "test_controller",
    "TestControllerType");
  EXPECT_EQ(1u, cm->get_loaded_controllers().size());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(
    0u,
    test_controller->internal_counter) <<
    "Update should not reached an unconfigured and deactivated controller";

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->configure());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->activate());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->update());
  EXPECT_EQ(1u, test_controller->internal_counter);

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->deactivate());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_node()->get_current_state().id());

  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->cleanup());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_node()->get_current_state().id());
}

std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> call_service_and_wait(
  rclcpp::Client<controller_manager_msgs::srv::ListControllers> & client,
  std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
  rclcpp::Executor & service_executor)
{
  EXPECT_TRUE(client.wait_for_service(std::chrono::milliseconds(500)));
  auto result = client.async_send_request(request);
  // Wait for the result.
  EXPECT_EQ(
    service_executor.spin_until_future_complete(result),
    rclcpp::FutureReturnCode::SUCCESS);
  return result.get();
}

TEST_F(TestControllerManager, list_controllers_srv) {
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    robot, executor,
    "test_controller_manager");

  // periodic calls to update
  auto timer = cm->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&controller_manager::ControllerManager::update, cm.get()));

  executor->add_node(cm);
  EXPECT_EQ(controller_interface::return_type::SUCCESS, cm->configure());

  auto future_handle = std::async(
    std::launch::async, [this]() -> void {
      executor->spin();
    });

  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllers>("list_controllers");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    0u,
    result->controller.size());


  auto test_controller = std::make_shared<test_controller::TestController>();
  static const std::string CONTROLLER_NAME = "test_controller";
  static const std::string CONTROLLER_TYPE = "TestControllerType";
  auto abstract_test_controller = cm->add_controller(
    test_controller, "test_controller",
    "TestControllerType");
  EXPECT_EQ(1u, cm->get_loaded_controllers().size());
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ(CONTROLLER_NAME, result->controller[0].name);
  ASSERT_EQ(CONTROLLER_TYPE, result->controller[0].type);
  ASSERT_EQ("inactive", result->controller[0].state);

  cm->switch_controller(
    {CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);


  cm->switch_controller(
    {}, {CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    1u,
    result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);

  cm->unload_controller(CONTROLLER_NAME);
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    0u,
    result->controller.size());
  executor->cancel();
}
