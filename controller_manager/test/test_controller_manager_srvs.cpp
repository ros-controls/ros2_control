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

  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

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
  // check chainable controller
  ASSERT_EQ(result->controller[0].state, "inactive");
  ASSERT_EQ(result->controller[0].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[0].required_command_interfaces.size(), 1u);
  ASSERT_EQ(result->controller[0].required_state_interfaces.size(), 2u);
  ASSERT_EQ(result->controller[0].is_chainable, true);
  ASSERT_EQ(result->controller[0].is_chained, false);
  ASSERT_EQ(result->controller[0].reference_interfaces.size(), 2u);
  ;
  ASSERT_EQ(result->controller[0].chain_connections.size(), 0u);
  // check test controller
  ASSERT_EQ(result->controller[1].state, "inactive");
  ASSERT_EQ(result->controller[1].claimed_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].required_command_interfaces.size(), 3u);
  ASSERT_EQ(result->controller[1].required_state_interfaces.size(), 2u);
  ASSERT_EQ(result->controller[1].is_chainable, false);
  ASSERT_EQ(result->controller[1].is_chained, false);
  ASSERT_EQ(result->controller[1].reference_interfaces.size(), 0u);
  ASSERT_EQ(result->controller[1].chain_connections.size(), 1u);
  // activate controllers
  cm_->switch_controller(
    {test_chainable_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  // get controller list after activate
  result = call_service_and_wait(*client, request, srv_executor);
  // check chainable controller
  ASSERT_EQ(result->controller[0].state, "active");
  ASSERT_EQ(result->controller[0].claimed_interfaces.size(), 1u);
  ASSERT_EQ(result->controller[0].is_chained, true);
  // check test controller
  ASSERT_EQ(result->controller[1].state, "active");
  ASSERT_EQ(result->controller[1].claimed_interfaces.size(), 3u);
  ASSERT_EQ(
    test_chainable_controller::TEST_CONTROLLER_NAME,
    result->controller[1].chain_connections[0].name);
  ASSERT_EQ(2u, result->controller[1].chain_connections[0].reference_interfaces.size());
  ASSERT_EQ("joint1/position", result->controller[1].chain_connections[0].reference_interfaces[0]);
  ASSERT_EQ("joint1/velocity", result->controller[1].chain_connections[0].reference_interfaces[1]);
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
}
