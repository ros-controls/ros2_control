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

using ::testing::_;
using ::testing::Return;

struct Strictness
{
  int strictness = STRICT;
  controller_interface::return_type expected_return;
  unsigned int expected_counter;
};
class TestControllerChainingWithControllerManager : public ControllerManagerFixture,
                                                    public testing::WithParamInterface<Strictness>
{
  void SetUp()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
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
  auto position_tracking_controller =
    std::make_shared<test_chainable_controller::TestChainableController>();

  // set controllers' names
  constexpr char PID_LEFT_WHEEL[] = "pid_left_wheel_controller";
  constexpr char PID_RIGHT_WHEEL[] = "pid_right_wheel_controller";
  constexpr char DIFF_DRIVE_CONTROLLER[] = "diff_drive_controller";
  constexpr char POSITION_TRACKING[] = "position_tracking_controller";

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
    position_tracking_controller, POSITION_TRACKING,
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

  // configure chainable controller and check exported interfaces
  cm_->configure_controller(PID_LEFT_WHEEL);
  EXPECT_EQ(cm_->reference_interface_map_.size(), 1u);
  EXPECT_EQ(cm_->claimed_reference_interface_map_.size(), 1u);
  EXPECT_EQ(cm_->claimed_reference_interface_map_["pid_left_wheel_controller/velocity"], false);
  EXPECT_EQ(
    pid_left_wheel_controller->get_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  //   pid_left_wheel_controller->get_node()->cleanup();

  // configure controllers - after chained controllers are configured their interfaces get exported
  //   EXPECT_TRUE(pid_lef)

  //   // configure controller
  //   cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  //   cm_->configure_controller(TEST_CONTROLLER2_NAME);
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";
  //   EXPECT_EQ(0u, test_controller2->internal_counter) << "Controller is not started";
  //
  //   EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());
  //
  //   // Start controller, will take effect at the end of the update function
  //   std::vector<std::string> start_controllers = {"fake_controller", TEST_CONTROLLER2_NAME};
  //   std::vector<std::string> stop_controllers = {};
  //   auto switch_future = std::async(
  //     std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
  //     start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));
  //
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_EQ(0u, test_controller2->internal_counter) << "Controller is started at the end of update";
  //   {
  //     ControllerManagerRunner cm_runner(this);
  //     EXPECT_EQ(test_param.expected_return, switch_future.get());
  //   }
  //
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_GE(test_controller2->internal_counter, test_param.expected_counter);
  //
  //   // Start the real test controller, will take effect at the end of the update function
  //   start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  //   stop_controllers = {};
  //   switch_future = std::async(
  //     std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
  //     start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));
  //
  //   ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
  //     << "switch_controller should be blocking until next update cycle";
  //
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is started at the end of update";
  //   {
  //     ControllerManagerRunner cm_runner(this);
  //     EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  //   }
  //   EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());
  //
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_GE(test_controller->internal_counter, 1u);
  //   auto last_internal_counter = test_controller->internal_counter;
  //
  //   // Stop controller, will take effect at the end of the update function
  //   start_controllers = {};
  //   stop_controllers = {test_controller::TEST_CONTROLLER_NAME};
  //   switch_future = std::async(
  //     std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
  //     start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));
  //
  //   ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
  //     << "switch_controller should be blocking until next update cycle";
  //
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));
  //   EXPECT_EQ(last_internal_counter + 1u, test_controller->internal_counter)
  //     << "Controller is stopped at the end of update, so it should have done one more update";
  //   {
  //     ControllerManagerRunner cm_runner(this);
  //     EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  //   }
  //
  //   EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());
  //   auto unload_future = std::async(
  //     std::launch::async, &controller_manager::ControllerManager::unload_controller, cm_,
  //     test_controller::TEST_CONTROLLER_NAME);
  //
  //   ASSERT_EQ(std::future_status::timeout, unload_future.wait_for(std::chrono::milliseconds(100)))
  //     << "unload_controller should be blocking until next update cycle";
  //   ControllerManagerRunner cm_runner(this);
  //   EXPECT_EQ(controller_interface::return_type::OK, unload_future.get());
  //
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  //   EXPECT_EQ(1, test_controller.use_count());
}

Strictness strict{STRICT, controller_interface::return_type::ERROR, 0u};
Strictness best_effort{BEST_EFFORT, controller_interface::return_type::OK, 1u};
INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerChainingWithControllerManager,
  testing::Values(strict, best_effort));
