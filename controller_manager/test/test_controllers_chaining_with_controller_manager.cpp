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
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_chainable_controller/test_chainable_controller.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;

class TestableControllerManager : public controller_manager::ControllerManager
{
  FRIEND_TEST(TestControllerChainingWithControllerManager, test_loading_chained_controllers);

public:
  TestableControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "")
  : controller_manager::ControllerManager(
      std::move(resource_manager), executor, manager_node_name, namespace_)
  {
  }
};

class TestControllerChainingWithControllerManager
: public ControllerManagerFixture<TestableControllerManager>,
  public testing::WithParamInterface<Strictness>
{
public:
  void SetUp()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<TestableControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        ros2_control_test_assets::diffbot_urdf, true, true),
      executor_, TEST_CM_NAME);
    run_updater_ = false;
  }
};

// The tests are implementing example of chained-control for DiffDrive robot shown here:
// https://github.com/ros-controls/roadmap/blob/9f32e215a84347aee0b519cb24d081f23bbbb224/design_drafts/cascade_control.md#motivation-purpose-and-use
// The controller have the names as stated in figure, but they are simply forwarding values without
// functionality that their name would suggest
TEST_P(TestControllerChainingWithControllerManager, test_loading_chained_controllers)
{
  const auto test_param = GetParam();
  auto pid_left_wheel_controller =
    std::make_shared<test_chainable_controller::TestChainableController>();
  auto pid_right_wheel_controller =
    std::make_shared<test_chainable_controller::TestChainableController>();
  auto diff_drive_controller =
    std::make_shared<test_chainable_controller::TestChainableController>();
  auto position_tracking_controller = std::make_shared<test_controller::TestController>();

  // set controllers' names
  constexpr char PID_LEFT_WHEEL[] = "pid_left_wheel_controller";
  constexpr char PID_RIGHT_WHEEL[] = "pid_right_wheel_controller";
  constexpr char DIFF_DRIVE_CONTROLLER[] = "diff_drive_controller";
  constexpr char POSITION_TRACKING_CONTROLLER[] = "position_tracking_controller";

  // configure Left Wheel controller
  controller_interface::InterfaceConfiguration pid_left_cmd_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}};
  controller_interface::InterfaceConfiguration pid_left_state_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}};
  pid_left_wheel_controller->set_command_interface_configuration(pid_left_cmd_ifs_cfg);
  pid_left_wheel_controller->set_state_interface_configuration(pid_left_state_ifs_cfg);
  pid_left_wheel_controller->set_reference_interface_names({"velocity"});

  // configure Left Wheel controller
  controller_interface::InterfaceConfiguration pid_right_cmd_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}};
  controller_interface::InterfaceConfiguration pid_right_state_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}};
  pid_right_wheel_controller->set_command_interface_configuration(pid_right_cmd_ifs_cfg);
  pid_right_wheel_controller->set_state_interface_configuration(pid_left_state_ifs_cfg);
  pid_right_wheel_controller->set_reference_interface_names({"velocity"});

  // configure Diff Drive controller
  controller_interface::InterfaceConfiguration diff_drive_cmd_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(PID_LEFT_WHEEL) + "/velocity", std::string(PID_RIGHT_WHEEL) + "/velocity"}};
  controller_interface::InterfaceConfiguration diff_drive_state_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"wheel_left/velocity", "wheel_right/velocity"}};
  diff_drive_controller->set_command_interface_configuration(diff_drive_cmd_ifs_cfg);
  diff_drive_controller->set_state_interface_configuration(diff_drive_state_ifs_cfg);
  diff_drive_controller->set_reference_interface_names({"vel_x", "vel_y", "rot_z"});

  // configure Position Tracking controller
  controller_interface::InterfaceConfiguration position_tracking_cmd_ifs_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {std::string(DIFF_DRIVE_CONTROLLER) + "/vel_x", std::string(DIFF_DRIVE_CONTROLLER) + "/vel_y"}};
  // in this simple example "vel_x" == "velocity left wheel" and "vel_y" == "velocity right wheel"
  position_tracking_controller->set_command_interface_configuration(position_tracking_cmd_ifs_cfg);

  // add all controllers
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  EXPECT_EQ(4u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, pid_left_wheel_controller.use_count());
  EXPECT_EQ(2, pid_right_wheel_controller.use_count());
  EXPECT_EQ(2, diff_drive_controller.use_count());
  EXPECT_EQ(2, position_tracking_controller.use_count());

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    pid_left_wheel_controller->get_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    pid_right_wheel_controller->get_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    diff_drive_controller->get_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    position_tracking_controller->get_state().id());

  // Store initial values of command interfaces
  size_t number_of_cmd_itfs = cm_->resource_manager_->command_interface_keys().size();

  // configure chainable controller and check exported interfaces
  cm_->configure_controller(PID_LEFT_WHEEL);
  EXPECT_EQ(
    pid_left_wheel_controller->get_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 1);
  for (const auto & interface : {"pid_left_wheel_controller/velocity"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  cm_->configure_controller(PID_RIGHT_WHEEL);
  EXPECT_EQ(
    pid_right_wheel_controller->get_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 2);
  for (const auto & interface : {"pid_right_wheel_controller/velocity"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  cm_->configure_controller(DIFF_DRIVE_CONTROLLER);
  EXPECT_EQ(
    diff_drive_controller->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 5);
  for (const auto & interface :
       {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y",
        "diff_drive_controller/rot_z"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  cm_->configure_controller(POSITION_TRACKING_CONTROLLER);
  EXPECT_EQ(
    position_tracking_controller->get_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 5);

  // set controllers to chained mode
  pid_left_wheel_controller->set_chained_mode(true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  pid_right_wheel_controller->set_chained_mode(true);
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  diff_drive_controller->set_chained_mode(true);
  EXPECT_TRUE(diff_drive_controller->is_in_chained_mode());

  // make reference interface command_interface_is_available
  cm_->resource_manager_->make_controller_reference_interfaces_available(PID_LEFT_WHEEL);
  for (const auto & interface : {"pid_left_wheel_controller/velocity"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  cm_->resource_manager_->make_controller_reference_interfaces_available(PID_RIGHT_WHEEL);
  for (const auto & interface : {"pid_right_wheel_controller/velocity"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  cm_->resource_manager_->make_controller_reference_interfaces_available(DIFF_DRIVE_CONTROLLER);
  for (const auto & interface :
       {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y",
        "diff_drive_controller/rot_z"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  EXPECT_THROW(
    cm_->resource_manager_->make_controller_reference_interfaces_available(
      POSITION_TRACKING_CONTROLLER),
    std::out_of_range);

  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);

  // activate controllers
  switch_test_controllers({PID_LEFT_WHEEL}, {}, test_param.strictness);
  EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed("wheel_left/velocity"));

  switch_test_controllers({PID_RIGHT_WHEEL}, {}, test_param.strictness);
  EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed("wheel_right/velocity"));

  // Diff-Drive Controller claims the reference interfaces of PID controllers
  switch_test_controllers({DIFF_DRIVE_CONTROLLER}, {}, test_param.strictness);
  for (const auto & interface :
       {"pid_left_wheel_controller/velocity", "pid_right_wheel_controller/velocity"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }

  // Position-Tracking Controller uses reference interfaces of Diff-Drive Controller
  switch_test_controllers({POSITION_TRACKING_CONTROLLER}, {}, test_param.strictness);
  for (const auto & interface : {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }
  for (const auto & interface : {"diff_drive_controller/rot_z"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerChainingWithControllerManager,
  testing::Values(strict, best_effort));
