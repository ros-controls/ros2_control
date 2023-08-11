// Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
// Copyright 2023 Christoph Hellmann Santos
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
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;

class TestControllerManagerWithNamespacedControllers
: public ControllerManagerFixture<controller_manager::ControllerManager>,
  public testing::WithParamInterface<Strictness>
{
public:
  void SetUp()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        ros2_control_test_assets::minimal_robot_urdf, true, true),
      executor_, TEST_CM_NAME, "/test_namespace");
    run_updater_ = false;
  }
};

TEST_P(TestControllerManagerWithNamespacedControllers, controller_in_absolute_namespace)
{
  auto test_controller = std::make_shared<test_controller::TestController>();
  auto test_controller2 = std::make_shared<test_controller::TestController>();
  constexpr char TEST_CONTROLLER1_NAME[] = "/device1/test_controller_name";
  constexpr char TEST_CONTROLLER2_NAME[] = "/device2/test_controller_name";
  cm_->add_controller(
    test_controller, TEST_CONTROLLER1_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller2, TEST_CONTROLLER2_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());

  // setup interface to claim from controllers
  controller_interface::InterfaceConfiguration cmd_itfs_cfg;
  cmd_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg.names.push_back(interface);
  }
  test_controller->set_command_interface_configuration(cmd_itfs_cfg);

  controller_interface::InterfaceConfiguration state_itfs_cfg;
  state_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES)
  {
    state_itfs_cfg.names.push_back(interface);
  }
  for (const auto & interface : ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES)
  {
    state_itfs_cfg.names.push_back(interface);
  }
  test_controller->set_state_interface_configuration(state_itfs_cfg);

  controller_interface::InterfaceConfiguration cmd_itfs_cfg2;
  cmd_itfs_cfg2.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg2.names.push_back(interface);
  }
  test_controller2->set_command_interface_configuration(cmd_itfs_cfg2);

  controller_interface::InterfaceConfiguration state_itfs_cfg2;
  state_itfs_cfg2.type = controller_interface::interface_configuration_type::ALL;
  test_controller2->set_state_interface_configuration(state_itfs_cfg2);

  // Check if namespace is set correctly
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller Manager namespace is '%s'",
    cm_->get_namespace());
  EXPECT_STREQ(cm_->get_namespace(), "/test_namespace");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 1 namespace is '%s'",
    test_controller->get_node()->get_namespace());
  EXPECT_STREQ(test_controller->get_node()->get_namespace(), "/device1");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 2 namespace is '%s'",
    test_controller2->get_node()->get_namespace());
  EXPECT_STREQ(test_controller2->get_node()->get_namespace(), "/device2");
}

TEST_P(TestControllerManagerWithNamespacedControllers, when_controller_is_defined_with_just_a_name_expect_it_relative_to_cm_namespace)
{
  auto test_controller = std::make_shared<test_controller::TestController>();
  auto test_controller2 = std::make_shared<test_controller::TestController>();
  constexpr char TEST_CONTROLLER1_NAME[] = "test_controller1_name";
  constexpr char TEST_CONTROLLER2_NAME[] = "test_controller2_name";
  cm_->add_controller(
    test_controller, TEST_CONTROLLER1_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller2, TEST_CONTROLLER2_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());


  // Check if namespace is set correctly
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller Manager namespace is '%s'",
    cm_->get_namespace());
  EXPECT_STREQ(cm_->get_namespace(), "/test_namespace");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 1 namespace is '%s'",
    test_controller->get_node()->get_namespace());
  EXPECT_STREQ(test_controller->get_node()->get_namespace(), "/test_namespace");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 2 namespace is '%s'",
    test_controller2->get_node()->get_namespace());
  EXPECT_STREQ(test_controller2->get_node()->get_namespace(), "/test_namespace");
}

TEST_P(TestControllerManagerWithNamespacedControllers, when_controller_has_relative_namespace_in_name_expect_it_under_cm_namspace_and_its_namespace)
{
  auto test_controller = std::make_shared<test_controller::TestController>();
  auto test_controller2 = std::make_shared<test_controller::TestController>();
  constexpr char TEST_CONTROLLER1_NAME[] = "device1/test_controller1_name";
  constexpr char TEST_CONTROLLER2_NAME[] = "device2/test_controller2_name";
  cm_->add_controller(
    test_controller, TEST_CONTROLLER1_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    test_controller2, TEST_CONTROLLER2_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(2u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());

  // setup interface to claim from controllers
  controller_interface::InterfaceConfiguration cmd_itfs_cfg;
  cmd_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg.names.push_back(interface);
  }
  test_controller->set_command_interface_configuration(cmd_itfs_cfg);

  controller_interface::InterfaceConfiguration state_itfs_cfg;
  state_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES)
  {
    state_itfs_cfg.names.push_back(interface);
  }
  for (const auto & interface : ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES)
  {
    state_itfs_cfg.names.push_back(interface);
  }
  test_controller->set_state_interface_configuration(state_itfs_cfg);

  controller_interface::InterfaceConfiguration cmd_itfs_cfg2;
  cmd_itfs_cfg2.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg2.names.push_back(interface);
  }
  test_controller2->set_command_interface_configuration(cmd_itfs_cfg2);

  controller_interface::InterfaceConfiguration state_itfs_cfg2;
  state_itfs_cfg2.type = controller_interface::interface_configuration_type::ALL;
  test_controller2->set_state_interface_configuration(state_itfs_cfg2);

  // Check if namespace is set correctly
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller Manager namespace is '%s'",
    cm_->get_namespace());
  EXPECT_STREQ(cm_->get_namespace(), "/test_namespace");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 1 namespace is '%s'",
    test_controller->get_node()->get_namespace());
  EXPECT_STREQ(test_controller->get_node()->get_namespace(), "/test_namespace/device1");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controll_manager_namespace"), "Controller 2 namespace is '%s'",
    test_controller2->get_node()->get_namespace());
  EXPECT_STREQ(test_controller2->get_node()->get_namespace(), "/test_namespace/device2");
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerManagerWithNamespacedControllers,
  testing::Values(strict, best_effort));
