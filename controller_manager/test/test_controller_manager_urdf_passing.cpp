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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_controller/test_controller.hpp"

const auto CONTROLLER_NAME = "test_controller";
using test_controller::TEST_CONTROLLER_CLASS_NAME;
using strvec = std::vector<std::string>;

class TestControllerManagerWithTestableCM;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerManagerWithTestableCM;

  FRIEND_TEST(
    TestControllerManagerWithTestableCM, initial_no_load_and_initialize_components_called);
  FRIEND_TEST(
    TestControllerManagerWithTestableCM, load_and_initialize_components_called_after_callback);
  FRIEND_TEST(
    TestControllerManagerWithTestableCM,
    expect_to_failure_when_invalid_urdf_is_given_and_be_able_to_submit_new_robot_description);

public:
  TestableControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "")
  : controller_manager::ControllerManager(
      std::move(resource_manager), executor, manager_node_name, node_namespace)
  {
  }
};

class TestControllerManagerWithTestableCM
: public ControllerManagerFixture<TestableControllerManager>,
  public testing::WithParamInterface<Strictness>
{
public:
  // create cm with no urdf
  TestControllerManagerWithTestableCM() : ControllerManagerFixture<TestableControllerManager>("") {}

  void prepare_controller()
  {
    ASSERT_FALSE(cm_->is_resource_manager_initialized());
    test_controller_ = std::make_shared<test_controller::TestController>();
    cm_->add_controller(test_controller_, CONTROLLER_NAME, TEST_CONTROLLER_CLASS_NAME);
    ASSERT_NE(test_controller_, nullptr);
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      test_controller_->get_lifecycle_state().id());
  }

  void configure_and_try_switch_that_returns_error()
  {
    // configure controller
    cm_->configure_controller(CONTROLLER_NAME);
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_->get_lifecycle_state().id());

    // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
    cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);

    switch_test_controllers(
      strvec{CONTROLLER_NAME}, strvec{}, STRICT, std::future_status::ready,
      controller_interface::return_type::ERROR);

    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_->get_lifecycle_state().id());
  }

  void try_successful_switch()
  {
    switch_test_controllers(
      strvec{CONTROLLER_NAME}, strvec{}, STRICT, std::future_status::timeout,
      controller_interface::return_type::OK);

    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_controller_->get_lifecycle_state().id());
  }

  std::shared_ptr<test_controller::TestController> test_controller_;
};

TEST_P(TestControllerManagerWithTestableCM, initial_no_load_and_initialize_components_called)
{
  ASSERT_FALSE(cm_->resource_manager_->are_components_initialized());
}

TEST_P(TestControllerManagerWithTestableCM, load_and_initialize_components_called_after_callback)
{
  ASSERT_FALSE(cm_->is_resource_manager_initialized());
  pass_robot_description_to_cm_and_rm();
  ASSERT_TRUE(cm_->is_resource_manager_initialized());
  // mimic callback
  auto msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_robot_urdf;
  cm_->robot_description_callback(msg);
  ASSERT_TRUE(cm_->resource_manager_->are_components_initialized());
}

TEST_P(
  TestControllerManagerWithTestableCM,
  expect_to_failure_when_invalid_urdf_is_given_and_be_able_to_submit_new_robot_description)
{
  ASSERT_FALSE(cm_->is_resource_manager_initialized());
  pass_robot_description_to_cm_and_rm(
    ros2_control_test_assets::minimal_robot_missing_command_keys_urdf);
  ASSERT_FALSE(cm_->is_resource_manager_initialized());
  // wrong urdf
  auto msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_uninitializable_robot_urdf;
  cm_->robot_description_callback(msg);
  ASSERT_FALSE(cm_->resource_manager_->are_components_initialized());
  // correct urdf
  msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_robot_urdf;
  cm_->robot_description_callback(msg);
  ASSERT_TRUE(cm_->resource_manager_->are_components_initialized());
}

TEST_P(
  TestControllerManagerWithTestableCM,
  when_starting_broadcaster_expect_error_before_rm_is_initialized_with_robot_description)
{
  prepare_controller();

  // setup state interfaces to claim from controllers
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
  test_controller_->set_state_interface_configuration(state_itfs_cfg);

  configure_and_try_switch_that_returns_error();

  pass_robot_description_to_cm_and_rm();
  ASSERT_TRUE(cm_->is_resource_manager_initialized());

  try_successful_switch();
}

TEST_P(
  TestControllerManagerWithTestableCM,
  when_starting_controller_expect_error_before_rm_is_initialized_with_robot_description)
{
  prepare_controller();

  // setup command interfaces to claim from controllers
  controller_interface::InterfaceConfiguration cmd_itfs_cfg;
  cmd_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg.names.push_back(interface);
  }
  test_controller_->set_command_interface_configuration(cmd_itfs_cfg);

  configure_and_try_switch_that_returns_error();

  pass_robot_description_to_cm_and_rm();
  ASSERT_TRUE(cm_->is_resource_manager_initialized());

  try_successful_switch();
}

TEST_P(
  TestControllerManagerWithTestableCM,
  when_starting_controller_expect_error_before_rm_is_initialized_after_some_time)
{
  prepare_controller();

  // setup state interfaces to claim from controllers
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
  test_controller_->set_state_interface_configuration(state_itfs_cfg);

  // setup command interfaces to claim from controllers
  controller_interface::InterfaceConfiguration cmd_itfs_cfg;
  cmd_itfs_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES)
  {
    cmd_itfs_cfg.names.push_back(interface);
  }
  test_controller_->set_command_interface_configuration(cmd_itfs_cfg);

  configure_and_try_switch_that_returns_error();

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  pass_robot_description_to_cm_and_rm();
  ASSERT_TRUE(cm_->is_resource_manager_initialized());

  try_successful_switch();
}

INSTANTIATE_TEST_SUITE_P(
  test_best_effort, TestControllerManagerWithTestableCM, testing::Values(best_effort));
