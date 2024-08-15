// Copyright 2020 PAL Robotics S.L.
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
#include <vector>

#include "controller_manager_test_common.hpp"

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_chainable_controller/test_chainable_controller.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::UnorderedElementsAre;

using ListControllers = controller_manager_msgs::srv::ListControllers;
using TestController = test_controller::TestController;
using TestChainableController = test_chainable_controller::TestChainableController;

TEST_F(TestControllerManagerSrvs, list_controller_types)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllerTypes>(
      "test_controller_manager/list_controller_types");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllerTypes::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  // Number depends on the controllers that exist on the system
  size_t controller_types = result->types.size();
  ASSERT_GE(controller_types, 1u);
  ASSERT_EQ(controller_types, result->base_classes.size());
  ASSERT_THAT(result->types, ::testing::Contains(test_controller::TEST_CONTROLLER_CLASS_NAME));
  ASSERT_THAT(
    result->base_classes, ::testing::Contains("controller_interface::ControllerInterface"));
  ASSERT_THAT(
    result->base_classes,
    ::testing::Contains("controller_interface::ChainableControllerInterface"));
}

TEST_F(TestControllerManagerSrvs, list_controllers_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(0u, result->controller.size());

  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity", "joint2/position"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(1u, cm_->get_loaded_controllers().size());
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ(test_controller::TEST_CONTROLLER_NAME, result->controller[0].name);
  ASSERT_EQ(test_controller::TEST_CONTROLLER_CLASS_NAME, result->controller[0].type);
  ASSERT_EQ("unconfigured", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_TRUE(result->controller[0].required_command_interfaces.empty());
  ASSERT_TRUE(result->controller[0].required_state_interfaces.empty());

  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);
  ASSERT_THAT(
    result->controller[0].claimed_interfaces,
    UnorderedElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cmd_cfg = {controller_interface::interface_configuration_type::ALL};
  test_controller->set_command_interface_configuration(cmd_cfg);
  state_cfg = {controller_interface::interface_configuration_type::ALL};
  test_controller->set_state_interface_configuration(state_cfg);
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);
  ASSERT_THAT(
    result->controller[0].claimed_interfaces,
    UnorderedElementsAre(
      "joint2/velocity", "joint3/velocity", "joint2/max_acceleration", "configuration/max_tcp_jerk",
      "joint1/position", "joint1/max_velocity"));
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/max_velocity", "joint1/position",
      "joint2/max_acceleration", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/position", "joint1/some_unlisted_interface",
      "joint1/velocity", "joint2/acceleration", "joint2/position", "joint2/velocity",
      "joint3/acceleration", "joint3/position", "joint3/velocity", "sensor1/velocity"));

  // Switch with a very low timeout 1 ns and it should fail as there is no enough time to switch
  ASSERT_EQ(
    controller_interface::return_type::ERROR,
    cm_->switch_controller(
      {}, {test_controller::TEST_CONTROLLER_NAME},
      controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
      rclcpp::Duration(0, 1)));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);
  ASSERT_THAT(
    result->controller[0].claimed_interfaces,
    UnorderedElementsAre(
      "joint2/velocity", "joint3/velocity", "joint2/max_acceleration", "configuration/max_tcp_jerk",
      "joint1/position", "joint1/max_velocity"));
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/max_velocity", "joint1/position",
      "joint2/max_acceleration", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/position", "joint1/some_unlisted_interface",
      "joint1/velocity", "joint2/acceleration", "joint2/position", "joint2/velocity",
      "joint3/acceleration", "joint3/position", "joint3/velocity", "sensor1/velocity"));

  // Try again with higher timeout
  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(3, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/max_velocity", "joint1/position",
      "joint2/max_acceleration", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    UnorderedElementsAre(
      "configuration/max_tcp_jerk", "joint1/position", "joint1/some_unlisted_interface",
      "joint1/velocity", "joint2/acceleration", "joint2/position", "joint2/velocity",
      "joint3/acceleration", "joint3/position", "joint3/velocity", "sensor1/velocity"));

  ASSERT_EQ(
    controller_interface::return_type::OK,
    cm_->unload_controller(test_controller::TEST_CONTROLLER_NAME));
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(0u, result->controller.size());
}

TEST_F(TestControllerManagerSrvs, list_chained_controllers_srv)
{
  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();
  // create chained controller
  auto test_chained_controller = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller->set_reference_interface_names({"joint1/position", "joint1/velocity"});
  // create non-chained controller
  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(test_chainable_controller::TEST_CONTROLLER_NAME) + "/joint1/position",
     std::string(test_chainable_controller::TEST_CONTROLLER_NAME) + "/joint1/velocity",
     "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  // add controllers
  cm_->add_controller(
    test_chained_controller, test_chainable_controller::TEST_CONTROLLER_NAME,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(result->controller[0].name, "test_chainable_controller_name");
  ASSERT_EQ(result->controller[0].type, "controller_manager/test_chainable_controller");
  ASSERT_EQ(result->controller[0].state, "unconfigured");
  ASSERT_EQ(result->controller[0].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].required_command_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].required_state_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].is_chainable, true);
  ASSERT_EQ(result->controller[0].is_chained, false);
  ASSERT_EQ(result->controller[0].reference_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].chain_connections.size(), 0u);
  // check test controller
  ASSERT_EQ(result->controller[1].name, "test_controller_name");
  ASSERT_EQ(result->controller[1].type, "controller_manager/test_controller");
  ASSERT_EQ(result->controller[1].state, "unconfigured");
  ASSERT_EQ(result->controller[1].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].required_command_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].required_state_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].is_chainable, false);
  ASSERT_EQ(result->controller[1].is_chained, false);
  ASSERT_EQ(result->controller[1].reference_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].chain_connections.size(), 0u);
  // configure controllers
  cm_->configure_controller(test_chainable_controller::TEST_CONTROLLER_NAME);
  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(2u, result->controller.size());
  // At this stage, the controllers are already reordered
  // check chainable controller
  ASSERT_EQ(result->controller[0].state, "inactive");
  ASSERT_EQ(result->controller[0].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].required_command_interfaces.size(), 3u);
  ASSERT_EQ(result->controller[0].required_state_interfaces.size(), 2u);
  ASSERT_EQ(result->controller[0].is_chainable, false);
  ASSERT_EQ(result->controller[0].is_chained, false);
  ASSERT_EQ(result->controller[0].reference_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].chain_connections.size(), 1u);

  // check test controller
  ASSERT_EQ(result->controller[1].state, "inactive");
  ASSERT_EQ(result->controller[1].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].required_command_interfaces.size(), 1u);
  ASSERT_EQ(result->controller[1].required_state_interfaces.size(), 2u);
  ASSERT_EQ(result->controller[1].is_chainable, true);
  ASSERT_EQ(result->controller[1].is_chained, false);
  ASSERT_EQ(result->controller[1].reference_interfaces.size(), 2u);
  ASSERT_EQ(result->controller[1].chain_connections.size(), 0u);
  ASSERT_EQ("joint1/position", result->controller[1].reference_interfaces[0]);
  ASSERT_EQ("joint1/velocity", result->controller[1].reference_interfaces[1]);
  // activate controllers
  auto res = cm_->switch_controller(
    {test_chainable_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(res, controller_interface::return_type::OK);
  // we should here wait for the first controller to be activated, i.e., for its reference
  // interface to become available (mail loop runs on 100 Hz) - so we check the status at least once
  while (result->controller[1].state != "active")
  {
    result = call_service_and_wait(*client, request, srv_executor);
  }
  res = cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(res, controller_interface::return_type::OK);
  // get controller list after activate
  result = call_service_and_wait(*client, request, srv_executor);
  // check test controller
  ASSERT_EQ(result->controller[0].state, "active");
  ASSERT_EQ(result->controller[0].claimed_interfaces.size(), 3u);
  // check chainable controller
  ASSERT_EQ(result->controller[1].state, "active");
  ASSERT_EQ(result->controller[1].claimed_interfaces.size(), 1u);
  ASSERT_EQ(result->controller[1].is_chained, true);
  ASSERT_EQ(
    test_chainable_controller::TEST_CONTROLLER_NAME,
    result->controller[0].chain_connections[0].name);
  ASSERT_EQ(2u, result->controller[0].chain_connections[0].reference_interfaces.size());
  ASSERT_EQ("joint1/position", result->controller[0].chain_connections[0].reference_interfaces[0]);
  ASSERT_EQ("joint1/velocity", result->controller[0].chain_connections[0].reference_interfaces[1]);
}

TEST_F(TestControllerManagerSrvs, reload_controller_libraries_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ReloadControllerLibraries>(
      "test_controller_manager/reload_controller_libraries");
  auto request =
    std::make_shared<controller_manager_msgs::srv::ReloadControllerLibraries::Request>();

  // Reload with no controllers running
  request->force_kill = false;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_TRUE(result->ok);

  // Add a controller, but unconfigured
  std::shared_ptr<TestController> test_controller =
    std::dynamic_pointer_cast<TestController>(cm_->load_controller(
      test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));

  // weak_ptr so the only controller shared_ptr instance is owned by the controller_manager and
  // can be completely destroyed before reloading the library
  std::weak_ptr<controller_interface::ControllerInterface> test_controller_weak(test_controller);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should have have a copy of this shared ptr";

  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  // Cleanup is not called from UNCONFIGURED: https://design.ros2.org/articles/node_lifecycle.html
  ASSERT_EQ(cleanup_calls, 0u);
  ASSERT_EQ(test_controller.use_count(), 0)
    << "No more references to the controller after reloading.";
  test_controller.reset();

  // Add a controller, but inactive
  test_controller = std::dynamic_pointer_cast<TestController>(cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));
  test_controller_weak = test_controller;
  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);

  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should have have a copy of this shared ptr";

  cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  ASSERT_EQ(cleanup_calls, 1u);
  ASSERT_EQ(test_controller.use_count(), 0)
    << "No more references to the controller after reloading.";
  test_controller.reset();

  test_controller = std::dynamic_pointer_cast<TestController>(cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));
  test_controller_weak = test_controller;
  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  // Start Controller
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());

  // Failed reload due to active controller
  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Cannot reload if controllers are running";
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should still have have a copy of "
       "this shared ptr, no unloading was performed";

  cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  // Force stop active controller
  request->force_kill = true;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);

  ASSERT_EQ(test_controller_weak.use_count(), 0)
    << "No more references to the controller after reloading.";
  ASSERT_EQ(cleanup_calls, 1u)
    << "Controller should have been stopped and cleaned up with force_kill = true";
}

TEST_F(TestControllerManagerSrvs, load_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::LoadController>(
      "test_controller_manager/load_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "There's no param specifying the type for " << request->name;
  rclcpp::Parameter controller_type_parameter(
    std::string(test_controller::TEST_CONTROLLER_NAME) + ".type",
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->set_parameter(controller_type_parameter);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}

TEST_F(TestControllerManagerSrvs, unload_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::UnloadController>(
      "test_controller_manager/unload_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;

  auto test_controller = std::make_shared<TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  EXPECT_EQ(0u, cm_->get_loaded_controllers().size());
}

TEST_F(TestControllerManagerSrvs, configure_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ConfigureController>(
      "test_controller_manager/configure_controller");
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_client =
    srv_node->create_client<controller_manager_msgs::srv::UnloadController>(
      "test_controller_manager/unload_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;

  auto test_controller = std::make_shared<TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    cm_->get_loaded_controllers()[0].c->get_state().id());
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());

  // now unload the controller and check the state
  auto unload_request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  unload_request->name = test_controller::TEST_CONTROLLER_NAME;
  ASSERT_TRUE(call_service_and_wait(*unload_client, unload_request, srv_executor, true)->ok);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  EXPECT_EQ(0u, cm_->get_loaded_controllers().size());
}

TEST_F(TestControllerManagerSrvs, list_sorted_chained_controllers)
{
  /// The simulated controller chaining is:
  /// test_controller_name -> chain_ctrl_5 -> chain_ctrl_4 -> chain_ctrl_3 -> chain_ctrl_2 ->
  /// chain_ctrl_1
  ///
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  static constexpr char TEST_CHAINED_CONTROLLER_3[] = "test_chainable_controller_name_3";
  static constexpr char TEST_CHAINED_CONTROLLER_4[] = "test_chainable_controller_name_4";
  static constexpr char TEST_CHAINED_CONTROLLER_5[] = "test_chainable_controller_name_5";
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_3 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position"}};
  test_chained_controller_3->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_3->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_3->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_4 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_3) + "/joint1/position"}};
  test_chained_controller_4->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_4->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_4->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_5 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_4) + "/joint1/position"}};
  test_chained_controller_5->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_5->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_5->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  // create non-chained controller
  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_5) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_5) + "/joint1/velocity", "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  // add controllers
  cm_->add_controller(
    test_chained_controller_4, TEST_CHAINED_CONTROLLER_4,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_3, TEST_CHAINED_CONTROLLER_3,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_5, TEST_CHAINED_CONTROLLER_5,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(6u, result->controller.size());
  ASSERT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_4);
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_1);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_3);
  ASSERT_EQ(result->controller[3].name, TEST_CHAINED_CONTROLLER_5);
  ASSERT_EQ(result->controller[4].name, TEST_CHAINED_CONTROLLER_2);
  // check test controller
  ASSERT_EQ(result->controller[5].name, "test_controller_name");

  // configure controllers
  for (const auto & controller :
       {TEST_CHAINED_CONTROLLER_4, TEST_CHAINED_CONTROLLER_3, TEST_CHAINED_CONTROLLER_5,
        TEST_CHAINED_CONTROLLER_2, TEST_CHAINED_CONTROLLER_1,
        test_controller::TEST_CONTROLLER_NAME})
  {
    cm_->configure_controller(controller);
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(6u, result->controller.size());

  // reordered controllers
  ASSERT_EQ(result->controller[0].name, "test_controller_name");
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_5);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_4);
  ASSERT_EQ(result->controller[3].name, TEST_CHAINED_CONTROLLER_3);
  ASSERT_EQ(result->controller[4].name, TEST_CHAINED_CONTROLLER_2);
  ASSERT_EQ(result->controller[5].name, TEST_CHAINED_CONTROLLER_1);
  RCLCPP_ERROR(srv_node->get_logger(), "Check successful!");
}

TEST_F(TestControllerManagerSrvs, list_sorted_complex_chained_controllers)
{
  /// The simulated controller chain branching is:
  /// test_controller_name -> chain_ctrl_7 -> chain_ctrl_6 -> chain_ctrl_2 -> chain_ctrl_1
  /// &
  /// chain_ctrl_6 -> chain_ctrl_5 -> chain_ctrl_4 -> chain_ctrl_3
  ///
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  static constexpr char TEST_CHAINED_CONTROLLER_3[] = "test_chainable_controller_name_3";
  static constexpr char TEST_CHAINED_CONTROLLER_4[] = "test_chainable_controller_name_4";
  static constexpr char TEST_CHAINED_CONTROLLER_5[] = "test_chainable_controller_name_5";
  static constexpr char TEST_CHAINED_CONTROLLER_6[] = "test_chainable_controller_name_6";
  static constexpr char TEST_CHAINED_CONTROLLER_7[] = "test_chainable_controller_name_7";
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity", "joint2/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_3 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/velocity"}};
  test_chained_controller_3->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_3->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_3->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_4 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_3) + "/joint2/velocity"}};
  test_chained_controller_4->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_4->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_4->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_5 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_4) + "/joint2/velocity"}};
  test_chained_controller_5->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_5->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_5->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_6 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_5) + "/joint2/velocity"}};
  test_chained_controller_6->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_6->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_6->set_reference_interface_names({"joint1/position", "joint2/velocity"});

  auto test_chained_controller_7 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_6) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_6) + "/joint2/velocity"}};
  test_chained_controller_7->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_7->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_7->set_reference_interface_names({"joint1/position", "joint2/velocity"});

  // create non-chained controller
  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_7) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_7) + "/joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint2/velocity"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  // add controllers
  cm_->add_controller(
    test_chained_controller_4, TEST_CHAINED_CONTROLLER_4,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_3, TEST_CHAINED_CONTROLLER_3,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_5, TEST_CHAINED_CONTROLLER_5,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_6, TEST_CHAINED_CONTROLLER_6,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_7, TEST_CHAINED_CONTROLLER_7,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(8u, result->controller.size());
  ASSERT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_4);
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_1);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_3);
  ASSERT_EQ(result->controller[3].name, TEST_CHAINED_CONTROLLER_5);
  ASSERT_EQ(result->controller[4].name, TEST_CHAINED_CONTROLLER_6);
  ASSERT_EQ(result->controller[5].name, TEST_CHAINED_CONTROLLER_2);
  // check test controller
  ASSERT_EQ(result->controller[6].name, "test_controller_name");
  ASSERT_EQ(result->controller[7].name, TEST_CHAINED_CONTROLLER_7);

  // configure controllers
  for (const auto & controller :
       {TEST_CHAINED_CONTROLLER_4, TEST_CHAINED_CONTROLLER_3, TEST_CHAINED_CONTROLLER_5,
        TEST_CHAINED_CONTROLLER_7, TEST_CHAINED_CONTROLLER_2, TEST_CHAINED_CONTROLLER_1,
        TEST_CHAINED_CONTROLLER_6, test_controller::TEST_CONTROLLER_NAME})
  {
    cm_->configure_controller(controller);
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(8u, result->controller.size());

  // reordered controllers
  ASSERT_EQ(result->controller[0].name, "test_controller_name");
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_7);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_6);

  auto get_ctrl_pos = [result](const std::string & controller_name) -> int64_t
  {
    auto it = std::find_if(
      result->controller.begin(), result->controller.end(), [controller_name](auto itf)
      { return (itf.name.find(controller_name) != std::string::npos); });
    return std::distance(result->controller.begin(), it);
  };
  auto ctrl_1_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_1);
  auto ctrl_2_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_2);
  auto ctrl_3_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_3);
  auto ctrl_4_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_4);
  auto ctrl_5_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_5);
  auto ctrl_6_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_6);

  // Extra check to see that they are index only after their parent controller (ctrl_6)
  ASSERT_GT(ctrl_3_pos, ctrl_6_pos);
  ASSERT_GT(ctrl_1_pos, ctrl_6_pos);

  // first branch
  ASSERT_GT(ctrl_2_pos, ctrl_6_pos);
  ASSERT_GT(ctrl_1_pos, ctrl_2_pos);

  // second branch
  ASSERT_GT(ctrl_5_pos, ctrl_6_pos);
  ASSERT_GT(ctrl_4_pos, ctrl_5_pos);
  ASSERT_GT(ctrl_3_pos, ctrl_4_pos);
}

TEST_F(TestControllerManagerSrvs, list_sorted_independent_chained_controllers)
{
  /// The simulated controller chaining is:
  /// test_controller_name_1 -> chain_ctrl_3 -> chain_ctrl_2 -> chain_ctrl_1
  /// &&
  /// test_controller_name_2 -> chain_ctrl_6 -> chain_ctrl_5 -> chain_ctrl_4
  /// &&
  /// test_controller_name_7 -> test_controller_name_8
  ///
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  static constexpr char TEST_CHAINED_CONTROLLER_3[] = "test_chainable_controller_name_3";
  static constexpr char TEST_CHAINED_CONTROLLER_4[] = "test_chainable_controller_name_4";
  static constexpr char TEST_CHAINED_CONTROLLER_5[] = "test_chainable_controller_name_5";
  static constexpr char TEST_CHAINED_CONTROLLER_6[] = "test_chainable_controller_name_6";
  static constexpr char TEST_CHAINED_CONTROLLER_7[] = "test_chainable_controller_name_7";
  static constexpr char TEST_CHAINED_CONTROLLER_8[] = "test_chainable_controller_name_8";
  static constexpr char TEST_CONTROLLER_1[] = "test_controller_name_1";
  static constexpr char TEST_CONTROLLER_2[] = "test_controller_name_2";

  // First chain
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_3 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position"}};
  test_chained_controller_3->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_3->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_3->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_controller_1 = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_3) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_3) + "/joint1/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller_1->set_command_interface_configuration(cmd_cfg);
  test_controller_1->set_state_interface_configuration(state_cfg);

  // Second chain
  auto test_chained_controller_4 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/velocity"}};
  test_chained_controller_4->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_4->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_4->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_5 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_4) + "/joint2/velocity"}};
  test_chained_controller_5->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_5->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_5->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_6 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_5) + "/joint2/velocity"}};
  test_chained_controller_6->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_6->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_6->set_reference_interface_names({"joint2/velocity"});

  auto test_controller_2 = std::make_shared<TestController>();
  cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_6) + "/joint2/velocity"}};
  state_cfg = {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/velocity"}};
  test_controller_2->set_command_interface_configuration(cmd_cfg);
  test_controller_2->set_state_interface_configuration(state_cfg);

  // Third chain
  auto test_chained_controller_7 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint3/velocity"}};
  test_chained_controller_7->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_7->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_7->set_reference_interface_names({"joint3/velocity"});

  auto test_chained_controller_8 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_7) + "/joint3/velocity"}};
  test_chained_controller_8->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_8->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_8->set_reference_interface_names({"joint3/velocity"});

  // add controllers
  /// @todo add controllers in random order
  /// For now, adding the ordered case to see that current sorting doesn't change order
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_6, TEST_CHAINED_CONTROLLER_6,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_7, TEST_CHAINED_CONTROLLER_7,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller_1, TEST_CONTROLLER_1, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_5, TEST_CHAINED_CONTROLLER_5,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_3, TEST_CHAINED_CONTROLLER_3,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_4, TEST_CHAINED_CONTROLLER_4,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller_2, TEST_CONTROLLER_2, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_8, TEST_CHAINED_CONTROLLER_8,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);

  // check chainable controller
  ASSERT_EQ(10u, result->controller.size());
  EXPECT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_2);
  EXPECT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_6);
  EXPECT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_1);
  EXPECT_EQ(result->controller[3].name, TEST_CHAINED_CONTROLLER_7);
  EXPECT_EQ(result->controller[4].name, TEST_CONTROLLER_1);

  EXPECT_EQ(result->controller[5].name, TEST_CHAINED_CONTROLLER_5);
  EXPECT_EQ(result->controller[6].name, TEST_CHAINED_CONTROLLER_3);
  EXPECT_EQ(result->controller[7].name, TEST_CHAINED_CONTROLLER_4);
  EXPECT_EQ(result->controller[8].name, TEST_CONTROLLER_2);
  EXPECT_EQ(result->controller[9].name, TEST_CHAINED_CONTROLLER_8);

  // configure controllers
  auto ctrls_order = {TEST_CHAINED_CONTROLLER_3, TEST_CHAINED_CONTROLLER_5,
                      TEST_CHAINED_CONTROLLER_1, TEST_CONTROLLER_1,
                      TEST_CHAINED_CONTROLLER_4, TEST_CONTROLLER_2,
                      TEST_CHAINED_CONTROLLER_2, TEST_CHAINED_CONTROLLER_6,
                      TEST_CHAINED_CONTROLLER_7, TEST_CHAINED_CONTROLLER_8};
  for (const auto & controller : ctrls_order)
  {
    cm_->configure_controller(controller);
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(10u, result->controller.size());

  auto get_ctrl_pos = [result](const std::string & controller_name) -> int64_t
  {
    auto it = std::find_if(
      result->controller.begin(), result->controller.end(), [controller_name](auto itf)
      { return (itf.name.find(controller_name) != std::string::npos); });
    return std::distance(result->controller.begin(), it);
  };
  auto ctrl_chain_1_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_1);
  auto ctrl_chain_2_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_2);
  auto ctrl_chain_3_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_3);
  auto ctrl_chain_4_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_4);
  auto ctrl_chain_5_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_5);
  auto ctrl_chain_6_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_6);
  auto ctrl_chain_7_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_7);
  auto ctrl_chain_8_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_8);

  auto ctrl_1_pos = get_ctrl_pos(TEST_CONTROLLER_1);
  auto ctrl_2_pos = get_ctrl_pos(TEST_CONTROLLER_2);

  // Extra check to see that they are indexed after their parent controller
  // first chain
  ASSERT_GT(ctrl_chain_1_pos, ctrl_chain_2_pos);
  ASSERT_GT(ctrl_chain_2_pos, ctrl_chain_3_pos);
  ASSERT_GT(ctrl_chain_3_pos, ctrl_1_pos);

  // second tree
  ASSERT_GT(ctrl_chain_4_pos, ctrl_chain_5_pos);
  ASSERT_GT(ctrl_chain_5_pos, ctrl_chain_6_pos);
  ASSERT_GT(ctrl_chain_6_pos, ctrl_2_pos);

  // third tree
  ASSERT_GT(ctrl_chain_7_pos, ctrl_chain_8_pos);
}

TEST_F(TestControllerManagerSrvs, list_large_number_of_controllers_with_chains)
{
  /// The simulated controller chaining is:
  /// test_controller_name_1 -> chain_ctrl_3 -> chain_ctrl_2 -> chain_ctrl_1
  /// &&
  /// test_controller_name_2 -> chain_ctrl_6 -> chain_ctrl_5 -> chain_ctrl_4
  /// &&
  /// test_controller_name_7 -> test_controller_name_8
  /// &&
  /// There are 100 more other basic controllers and 100 more different broadcasters to check for
  /// crashing
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces
  /// exported from the controller B (or) the controller B is utilizing the expected interfaces
  /// exported from the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  static constexpr char TEST_CHAINED_CONTROLLER_3[] = "test_chainable_controller_name_3";
  static constexpr char TEST_CHAINED_CONTROLLER_4[] = "test_chainable_controller_name_4";
  static constexpr char TEST_CHAINED_CONTROLLER_5[] = "test_chainable_controller_name_5";
  static constexpr char TEST_CHAINED_CONTROLLER_6[] = "test_chainable_controller_name_6";
  static constexpr char TEST_CHAINED_CONTROLLER_7[] = "test_chainable_controller_name_7";
  static constexpr char TEST_CHAINED_CONTROLLER_8[] = "test_chainable_controller_name_8";
  static constexpr char TEST_CHAINED_CONTROLLER_9[] = "test_chainable_controller_name_9";
  static constexpr char TEST_CONTROLLER_1[] = "test_controller_name_1";
  static constexpr char TEST_CONTROLLER_2[] = "test_controller_name_2";

  // First chain
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_3 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position"}};
  test_chained_controller_3->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_3->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_3->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_controller_1 = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_3) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_3) + "/joint1/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller_1->set_command_interface_configuration(cmd_cfg);
  test_controller_1->set_state_interface_configuration(state_cfg);

  // Second chain
  auto test_chained_controller_4 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/velocity"}};
  test_chained_controller_4->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_4->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_4->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_5 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_4) + "/joint2/velocity"}};
  test_chained_controller_5->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_5->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_5->set_reference_interface_names({"joint2/velocity"});

  auto test_chained_controller_6 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_5) + "/joint2/velocity"}};
  test_chained_controller_6->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_6->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_6->set_reference_interface_names({"joint2/velocity"});

  auto test_controller_2 = std::make_shared<TestController>();
  cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_6) + "/joint2/velocity"}};
  state_cfg = {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/velocity"}};
  test_controller_2->set_command_interface_configuration(cmd_cfg);
  test_controller_2->set_state_interface_configuration(state_cfg);

  // Third chain
  auto test_chained_controller_7 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint3/velocity"}};
  test_chained_controller_7->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_7->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_7->set_reference_interface_names({"joint3/velocity"});

  auto test_chained_controller_8 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_7) + "/joint3/velocity"}};
  test_chained_controller_8->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_8->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_8->set_reference_interface_names({"joint3/velocity"});

  auto test_chained_controller_9 = std::make_shared<TestChainableController>();
  cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint2/max_acceleration"}};
  test_chained_controller_9->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_9->set_state_interface_configuration(state_cfg);

  unsigned int num_of_random_broadcasters = 100;
  unsigned int num_of_random_controllers = 100;
  std::vector<std::string> chained_ref_interfaces;
  for (size_t i = 0; i < num_of_random_controllers; i++)
  {
    chained_ref_interfaces.push_back("ref_" + std::to_string(i) + "/joint_2/acceleration");
  }
  test_chained_controller_9->set_reference_interface_names(chained_ref_interfaces);
  std::unordered_map<std::string, std::shared_ptr<TestController>> random_controllers_list;
  for (size_t i = 0; i < num_of_random_broadcasters; i++)
  {
    auto controller_name = "test_broadcaster_" + std::to_string(i);
    random_controllers_list[controller_name] = std::make_shared<TestController>();
    random_controllers_list[controller_name]->set_state_interface_configuration(state_cfg);
  }
  for (size_t i = 0; i < num_of_random_controllers; i++)
  {
    auto controller_name = "test_random_controllers_" + std::to_string(i);
    RCLCPP_ERROR(srv_node->get_logger(), "Initializing controller : %s !", controller_name.c_str());
    random_controllers_list[controller_name] = std::make_shared<TestController>();
    random_controllers_list[controller_name]->set_state_interface_configuration(state_cfg);
    random_controllers_list[controller_name]->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string(TEST_CHAINED_CONTROLLER_9) + std::string("/ref_") + std::to_string(i) +
        std::string("/joint_2/acceleration")}});
  }

  // add controllers
  /// @todo add controllers in random order
  /// For now, adding the ordered case to see that current sorting doesn't change order
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_6, TEST_CHAINED_CONTROLLER_6,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_7, TEST_CHAINED_CONTROLLER_7,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller_1, TEST_CONTROLLER_1, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_5, TEST_CHAINED_CONTROLLER_5,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_3, TEST_CHAINED_CONTROLLER_3,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_4, TEST_CHAINED_CONTROLLER_4,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller_2, TEST_CONTROLLER_2, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_8, TEST_CHAINED_CONTROLLER_8,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_9, TEST_CHAINED_CONTROLLER_9,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  {
    ControllerManagerRunner cm_runner(this);
    for (auto random_ctrl : random_controllers_list)
    {
      cm_->add_controller(
        random_ctrl.second, random_ctrl.first,
        test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    }
  }

  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);

  // check chainable controller
  ASSERT_EQ(
    11u + num_of_random_broadcasters + num_of_random_controllers, result->controller.size());
  EXPECT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_2);
  EXPECT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_6);
  EXPECT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_1);
  EXPECT_EQ(result->controller[3].name, TEST_CHAINED_CONTROLLER_7);
  EXPECT_EQ(result->controller[4].name, TEST_CONTROLLER_1);

  EXPECT_EQ(result->controller[5].name, TEST_CHAINED_CONTROLLER_5);
  EXPECT_EQ(result->controller[6].name, TEST_CHAINED_CONTROLLER_3);
  EXPECT_EQ(result->controller[7].name, TEST_CHAINED_CONTROLLER_4);
  EXPECT_EQ(result->controller[8].name, TEST_CONTROLLER_2);
  EXPECT_EQ(result->controller[9].name, TEST_CHAINED_CONTROLLER_8);

  // configure controllers
  auto ctrls_order = {
    TEST_CHAINED_CONTROLLER_3, TEST_CHAINED_CONTROLLER_5, TEST_CHAINED_CONTROLLER_9,
    TEST_CHAINED_CONTROLLER_1, TEST_CONTROLLER_1,         TEST_CHAINED_CONTROLLER_4,
    TEST_CONTROLLER_2,         TEST_CHAINED_CONTROLLER_2, TEST_CHAINED_CONTROLLER_6,
    TEST_CHAINED_CONTROLLER_7, TEST_CHAINED_CONTROLLER_8};
  {
    ControllerManagerRunner cm_runner(this);
    for (const auto & controller : ctrls_order)
    {
      cm_->configure_controller(controller);
    }

    for (auto random_ctrl : random_controllers_list)
    {
      cm_->configure_controller(random_ctrl.first);
    }
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(
    11u + num_of_random_broadcasters + num_of_random_controllers, result->controller.size());

  auto get_ctrl_pos = [result](const std::string & controller_name) -> int64_t
  {
    auto it = std::find_if(
      result->controller.begin(), result->controller.end(), [controller_name](auto itf)
      { return (itf.name.find(controller_name) != std::string::npos); });
    return std::distance(result->controller.begin(), it);
  };
  auto ctrl_chain_1_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_1);
  auto ctrl_chain_2_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_2);
  auto ctrl_chain_3_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_3);
  auto ctrl_chain_4_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_4);
  auto ctrl_chain_5_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_5);
  auto ctrl_chain_6_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_6);
  auto ctrl_chain_7_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_7);
  auto ctrl_chain_8_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_8);
  auto ctrl_chain_9_pos = get_ctrl_pos(TEST_CHAINED_CONTROLLER_9);

  auto ctrl_1_pos = get_ctrl_pos(TEST_CONTROLLER_1);
  auto ctrl_2_pos = get_ctrl_pos(TEST_CONTROLLER_2);

  // Extra check to see that they are indexed after their parent controller
  // first chain
  ASSERT_GT(ctrl_chain_1_pos, ctrl_chain_2_pos);
  ASSERT_GT(ctrl_chain_2_pos, ctrl_chain_3_pos);
  ASSERT_GT(ctrl_chain_3_pos, ctrl_1_pos);

  // second tree
  ASSERT_GT(ctrl_chain_4_pos, ctrl_chain_5_pos);
  ASSERT_GT(ctrl_chain_5_pos, ctrl_chain_6_pos);
  ASSERT_GT(ctrl_chain_6_pos, ctrl_2_pos);

  // third tree
  ASSERT_GT(ctrl_chain_7_pos, ctrl_chain_8_pos);

  for (size_t i = 0; i < num_of_random_controllers; i++)
  {
    auto controller_name = "test_random_controllers_" + std::to_string(i);
    ASSERT_GT(ctrl_chain_9_pos, get_ctrl_pos(controller_name));
  }
  RCLCPP_INFO(srv_node->get_logger(), "Check successful!");
}

TEST_F(TestControllerManagerSrvs, list_sorted_large_chained_controller_tree)
{
  /// The simulated controller chain is like every joint has its own controller exposing interfaces
  /// and then a controller chain using those interfaces
  ///
  /// There are 20 more broadcasters + 20 more normal controllers for complexity
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  const unsigned int joints_count = 20;

  static constexpr char JOINT_CONTROLLER_PREFIX[] = "test_chainable_controller_name_joint_";
  static constexpr char FWD_CONTROLLER_PREFIX[] = "forward_controller_joint_";
  static constexpr char JOINT_SENSOR_BROADCASTER_PREFIX[] = "test_broadcaster_joint_";
  std::vector<std::string> controllers_list;
  std::vector<std::string> fwd_joint_position_interfaces_list;
  std::vector<std::string> fwd_joint_velocity_interfaces_list;
  std::vector<std::string> fwd_joint_position_ref_interfaces_list;
  std::vector<std::string> fwd_joint_velocity_ref_interfaces_list;
  std::unordered_map<std::string, std::shared_ptr<TestChainableController>>
    random_chainable_controllers_list;
  std::unordered_map<std::string, std::shared_ptr<TestController>> random_controllers_list;
  for (size_t i = 0; i < joints_count; i++)
  {
    controller_interface::InterfaceConfiguration chained_cmd_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {"joint" + std::to_string(i) + "/position"}};
    controller_interface::InterfaceConfiguration chained_state_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {"joint" + std::to_string(i) + "/position", "joint" + std::to_string(i) + "/velocity"}};
    // Joint controller
    const std::string controller_name = JOINT_CONTROLLER_PREFIX + std::to_string(i);
    random_chainable_controllers_list[controller_name] =
      std::make_shared<TestChainableController>();
    random_chainable_controllers_list[controller_name]->set_state_interface_configuration(
      chained_state_cfg);
    random_chainable_controllers_list[controller_name]->set_command_interface_configuration(
      chained_cmd_cfg);
    random_chainable_controllers_list[controller_name]->set_reference_interface_names(
      chained_state_cfg.names);
    controllers_list.push_back(controller_name);

    // Forward Joint interfaces controller
    fwd_joint_position_interfaces_list.push_back(
      std::string(controller_name) + "/joint" + std::to_string(i) + "/position");
    fwd_joint_velocity_interfaces_list.push_back(
      std::string(controller_name) + "/joint" + std::to_string(i) + "/velocity");
    const std::string fwd_controller_name = FWD_CONTROLLER_PREFIX + std::to_string(i);
    random_chainable_controllers_list[fwd_controller_name] =
      std::make_shared<TestChainableController>();
    random_chainable_controllers_list[fwd_controller_name]->set_state_interface_configuration(
      chained_state_cfg);
    random_chainable_controllers_list[fwd_controller_name]->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {fwd_joint_position_interfaces_list.back(), fwd_joint_velocity_interfaces_list.back()}});
    random_chainable_controllers_list[fwd_controller_name]->set_reference_interface_names(
      chained_state_cfg.names);
    fwd_joint_position_ref_interfaces_list.push_back(
      std::string(fwd_controller_name) + "/" + fwd_joint_position_interfaces_list.back());
    fwd_joint_velocity_ref_interfaces_list.push_back(
      std::string(fwd_controller_name) + "/" + fwd_joint_velocity_interfaces_list.back());
    controllers_list.push_back(fwd_controller_name);

    // Add a broadcaster for every joint assuming it as a sensor (just for the tests)
    const std::string broadcaster_name = JOINT_SENSOR_BROADCASTER_PREFIX + std::to_string(i);
    random_controllers_list[broadcaster_name] = std::make_shared<TestController>();
    random_controllers_list[broadcaster_name]->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {"joint" + std::to_string(i) + "/torque", "joint" + std::to_string(i) + "/torque"}});
    controllers_list.push_back(broadcaster_name);
  }

  // create set of chained controllers
  static constexpr char POSITION_REFERENCE_CONTROLLER[] = "position_reference_chainable_controller";
  static constexpr char VELOCITY_REFERENCE_CONTROLLER[] = "velocity_reference_chainable_controller";
  static constexpr char HIGHER_LEVEL_REFERENCE_CONTROLLER[] = "task_level_controller";

  // Position reference controller
  random_chainable_controllers_list[POSITION_REFERENCE_CONTROLLER] =
    std::make_shared<TestChainableController>();
  random_chainable_controllers_list[POSITION_REFERENCE_CONTROLLER]
    ->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       fwd_joint_position_ref_interfaces_list});
  random_chainable_controllers_list[POSITION_REFERENCE_CONTROLLER]->set_reference_interface_names(
    {"joint/position"});
  controllers_list.push_back(POSITION_REFERENCE_CONTROLLER);

  // Velocity reference controller
  random_chainable_controllers_list[VELOCITY_REFERENCE_CONTROLLER] =
    std::make_shared<TestChainableController>();
  random_chainable_controllers_list[VELOCITY_REFERENCE_CONTROLLER]
    ->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       fwd_joint_velocity_ref_interfaces_list});
  random_chainable_controllers_list[VELOCITY_REFERENCE_CONTROLLER]->set_reference_interface_names(
    {"joint/velocity"});
  controllers_list.push_back(VELOCITY_REFERENCE_CONTROLLER);

  // Higher level task level controller
  random_controllers_list[HIGHER_LEVEL_REFERENCE_CONTROLLER] = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(POSITION_REFERENCE_CONTROLLER) + "/joint/position",
     std::string(VELOCITY_REFERENCE_CONTROLLER) + "/joint/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  random_controllers_list[HIGHER_LEVEL_REFERENCE_CONTROLLER]->set_command_interface_configuration(
    cmd_cfg);
  random_controllers_list[HIGHER_LEVEL_REFERENCE_CONTROLLER]->set_state_interface_configuration(
    state_cfg);
  controllers_list.push_back(HIGHER_LEVEL_REFERENCE_CONTROLLER);

  const unsigned int num_of_random_broadcasters = 20;
  const unsigned int num_of_random_controllers = 20;
  for (size_t i = 0; i < num_of_random_broadcasters; i++)
  {
    auto controller_name = "test_broadcaster_" + std::to_string(i);
    random_controllers_list[controller_name] = std::make_shared<TestController>();
    random_controllers_list[controller_name]->set_state_interface_configuration(state_cfg);
    controllers_list.push_back(controller_name);
  }
  for (size_t i = 0; i < num_of_random_controllers; i++)
  {
    auto controller_name = "test_random_reference_controllers_" + std::to_string(i);
    RCLCPP_ERROR(srv_node->get_logger(), "Initializing controller : %s !", controller_name.c_str());
    random_controllers_list[controller_name] = std::make_shared<TestController>();
    random_controllers_list[controller_name]->set_state_interface_configuration(state_cfg);
    random_controllers_list[controller_name]->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string("ref_") + std::to_string(i) + std::string("/joint_2/acceleration")}});
    controllers_list.push_back(controller_name);
  }

  // Now shuffle the list to be able to configure controller later randomly
  std::random_shuffle(controllers_list.begin(), controllers_list.end());

  {
    ControllerManagerRunner cm_runner(this);
    for (auto random_chain_ctrl : random_chainable_controllers_list)
    {
      cm_->add_controller(
        random_chain_ctrl.second, random_chain_ctrl.first,
        test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    }
    for (auto random_ctrl : random_controllers_list)
    {
      cm_->add_controller(
        random_ctrl.second, random_ctrl.first,
        test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    }
  }

  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);

  // check chainable controller
  ASSERT_EQ(controllers_list.size(), result->controller.size());

  // configure controllers
  {
    ControllerManagerRunner cm_runner(this);
    for (auto random_ctrl : controllers_list)
    {
      cm_->configure_controller(random_ctrl);
    }
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(controllers_list.size(), result->controller.size());

  auto get_ctrl_pos = [result](const std::string & controller_name) -> int64_t
  {
    auto it = std::find_if(
      result->controller.begin(), result->controller.end(), [controller_name](auto itf)
      { return (itf.name.find(controller_name) != std::string::npos); });
    return std::distance(result->controller.begin(), it);
  };

  // Check the controller indexing
  auto pos_ref_pos = get_ctrl_pos(POSITION_REFERENCE_CONTROLLER);
  auto vel_ref_pos = get_ctrl_pos(VELOCITY_REFERENCE_CONTROLLER);
  auto task_level_ctrl_pos = get_ctrl_pos(HIGHER_LEVEL_REFERENCE_CONTROLLER);
  ASSERT_GT(vel_ref_pos, task_level_ctrl_pos);
  ASSERT_GT(pos_ref_pos, task_level_ctrl_pos);

  for (size_t i = 0; i < joints_count; i++)
  {
    const std::string controller_name = JOINT_CONTROLLER_PREFIX + std::to_string(i);
    const std::string fwd_controller_name = FWD_CONTROLLER_PREFIX + std::to_string(i);

    ASSERT_GT(get_ctrl_pos(fwd_controller_name), pos_ref_pos);
    ASSERT_GT(get_ctrl_pos(fwd_controller_name), vel_ref_pos);
    ASSERT_GT(get_ctrl_pos(controller_name), pos_ref_pos);
    ASSERT_GT(get_ctrl_pos(controller_name), vel_ref_pos);
  }
  RCLCPP_INFO(srv_node->get_logger(), "Check successful!");
}

TEST_F(TestControllerManagerSrvs, activate_chained_controllers_one_by_one)
{
  /// The simulated controller chaining is:
  /// test_controller_name -> chain_ctrl_2 -> chain_ctrl_1
  ///
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  // create non-chained controller
  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/velocity", "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  // add controllers
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(3u, result->controller.size());
  ASSERT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_1);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_2);
  // check test controller
  ASSERT_EQ(result->controller[1].name, "test_controller_name");

  // configure controllers
  for (const auto & controller :
       {TEST_CHAINED_CONTROLLER_1, test_controller::TEST_CONTROLLER_NAME,
        TEST_CHAINED_CONTROLLER_2})
  {
    cm_->configure_controller(controller);
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(3u, result->controller.size());

  // reordered controllers
  ASSERT_EQ(result->controller[0].name, "test_controller_name");
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_2);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_1);

  // activate controllers one by one
  auto res1 = cm_->switch_controller(
    {TEST_CHAINED_CONTROLLER_1}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(res1, controller_interface::return_type::OK);
  auto res2 = cm_->switch_controller(
    {TEST_CHAINED_CONTROLLER_2}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(res2, controller_interface::return_type::OK);
  auto res3 = cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(res3, controller_interface::return_type::OK);

  RCLCPP_ERROR(srv_node->get_logger(), "Check successful!");
}

TEST_F(TestControllerManagerSrvs, activate_chained_controllers_all_at_once)
{
  /// The simulated controller chaining is:
  /// test_controller_name -> chain_ctrl_2 -> chain_ctrl_1
  ///
  /// NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported
  /// from the controller B (or) the controller B is utilizing the expected interfaces exported from
  /// the controller A

  // create server client and request
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<ListControllers>::SharedPtr client =
    srv_node->create_client<ListControllers>("test_controller_manager/list_controllers");
  auto request = std::make_shared<ListControllers::Request>();

  // create set of chained controllers
  static constexpr char TEST_CHAINED_CONTROLLER_1[] = "test_chainable_controller_name_1";
  static constexpr char TEST_CHAINED_CONTROLLER_2[] = "test_chainable_controller_name_2";
  auto test_chained_controller_1 = std::make_shared<TestChainableController>();
  controller_interface::InterfaceConfiguration chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position"}};
  controller_interface::InterfaceConfiguration chained_state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_chained_controller_1->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_1->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_1->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  auto test_chained_controller_2 = std::make_shared<TestChainableController>();
  chained_cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_1) + "/joint1/position"}};
  test_chained_controller_2->set_command_interface_configuration(chained_cmd_cfg);
  test_chained_controller_2->set_state_interface_configuration(chained_state_cfg);
  test_chained_controller_2->set_reference_interface_names({"joint1/position", "joint1/velocity"});

  // create non-chained controller
  auto test_controller = std::make_shared<TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/position",
     std::string(TEST_CHAINED_CONTROLLER_2) + "/joint1/velocity", "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  // add controllers
  cm_->add_controller(
    test_chained_controller_1, TEST_CHAINED_CONTROLLER_1,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_chained_controller_2, TEST_CHAINED_CONTROLLER_2,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  // get controller list before configure
  auto result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(3u, result->controller.size());
  ASSERT_EQ(result->controller[0].name, TEST_CHAINED_CONTROLLER_1);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_2);
  // check test controller
  ASSERT_EQ(result->controller[1].name, "test_controller_name");

  // configure controllers
  for (const auto & controller :
       {TEST_CHAINED_CONTROLLER_1, test_controller::TEST_CONTROLLER_NAME,
        TEST_CHAINED_CONTROLLER_2})
  {
    cm_->configure_controller(controller);
  }

  // get controller list after configure
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(3u, result->controller.size());

  // reordered controllers
  ASSERT_EQ(result->controller[0].name, "test_controller_name");
  ASSERT_EQ(result->controller[1].name, TEST_CHAINED_CONTROLLER_2);
  ASSERT_EQ(result->controller[2].name, TEST_CHAINED_CONTROLLER_1);

  // activate controllers all at once
  auto res = cm_->switch_controller(
    {TEST_CHAINED_CONTROLLER_1, TEST_CHAINED_CONTROLLER_2, test_controller::TEST_CONTROLLER_NAME},
    {}, controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
    rclcpp::Duration(0, 0));
  ASSERT_EQ(res, controller_interface::return_type::OK);

  RCLCPP_ERROR(srv_node->get_logger(), "Check successful!");
}
