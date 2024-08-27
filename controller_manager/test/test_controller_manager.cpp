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

#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;

class TestControllerManagerWithStrictness
: public ControllerManagerFixture<controller_manager::ControllerManager>,
  public testing::WithParamInterface<Strictness>
{
};

class TestControllerManagerRobotDescription
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
};

TEST_F(TestControllerManagerRobotDescription, controller_robot_description_update)
{
  auto test_controller = std::make_shared<test_controller::TestController>();
  auto test_controller2 = std::make_shared<test_controller::TestController>();
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(ros2_control_test_assets::minimal_robot_urdf, test_controller->get_robot_description());

  // Now change the robot description and then load a new controller and see if the new controller
  // gets the new description and the old controller still maintains the configuration
  auto msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_robot_missing_state_keys_urdf;
  cm_->robot_description_callback(msg);
  cm_->add_controller(
    test_controller2, test_controller::TEST_CONTROLLER2_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(ros2_control_test_assets::minimal_robot_urdf, test_controller->get_robot_description());
  ASSERT_EQ(
    ros2_control_test_assets::minimal_robot_missing_state_keys_urdf,
    test_controller2->get_robot_description());
}

TEST_P(TestControllerManagerWithStrictness, controller_lifecycle)
{
  const auto test_param = GetParam();
  auto test_controller = std::make_shared<test_controller::TestController>();
  auto test_controller2 = std::make_shared<test_controller::TestController>();
  constexpr char TEST_CONTROLLER2_NAME[] = "test_controller2_name";
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
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
    rclcpp::get_logger("test_controller_manager"), "Controller Manager namespace is '%s'",
    cm_->get_namespace());
  EXPECT_STREQ(cm_->get_namespace(), "/");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controller_manager"), "Controller 1 namespace is '%s'",
    test_controller->get_node()->get_namespace());
  EXPECT_STREQ(test_controller->get_node()->get_namespace(), "/");
  RCLCPP_INFO(
    rclcpp::get_logger("test_controller_manager"), "Controller 2 namespace is '%s'",
    test_controller2->get_node()->get_namespace());
  EXPECT_STREQ(test_controller2->get_node()->get_namespace(), "/");

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter)
    << "Update should not reach an unconfigured controller";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_state().id());

  // configure controller
  {
    ControllerManagerRunner cm_runner(this);
    cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
    cm_->configure_controller(TEST_CONTROLLER2_NAME);
  }
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";
  EXPECT_EQ(0u, test_controller2->internal_counter) << "Controller is not started";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_state().id());

  // Start controller, will take effect at the end of the update function
  std::vector<std::string> start_controllers = {"fake_controller", TEST_CONTROLLER2_NAME};
  std::vector<std::string> stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller2->internal_counter) << "Controller is started at the end of update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(test_param.expected_return, switch_future.get());
  }

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_GE(test_controller2->internal_counter, test_param.expected_counter);

  // Start the real test controller, will take effect at the end of the update function
  start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  stop_controllers = {};
  switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is started at the end of update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_lifecycle_state().id());

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_GE(test_controller->internal_counter, 1u);
  size_t last_internal_counter = test_controller->internal_counter;

  // Stop controller, will take effect at the end of the update function
  start_controllers = {};
  stop_controllers = {test_controller::TEST_CONTROLLER_NAME};
  switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, test_param.strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(last_internal_counter + 1u, test_controller->internal_counter)
    << "Controller is stopped at the end of update, so it should have done one more update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_state().id());
  auto unload_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::unload_controller, cm_,
    test_controller::TEST_CONTROLLER_NAME);

  ASSERT_EQ(std::future_status::timeout, unload_future.wait_for(std::chrono::milliseconds(100)))
    << "unload_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, unload_future.get());

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_state().id());
  EXPECT_EQ(1, test_controller.use_count());
}

TEST_P(TestControllerManagerWithStrictness, per_controller_update_rate)
{
  auto strictness = GetParam().strictness;
  auto test_controller = std::make_shared<test_controller::TestController>();
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter)
    << "Update should not reach an unconfigured controller";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_state().id());

  test_controller->get_node()->set_parameter({"update_rate", 4});
  // configure controller
  {
    ControllerManagerRunner cm_runner(this);
    cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  }
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_state().id());

  // Start controller, will take effect at the end of the update function
  std::vector<std::string> start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  std::vector<std::string> stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is started at the end of update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_lifecycle_state().id());

  // As the controller frequency is 4Hz, it needs to pass 25 iterations for 1 update cycle
  for (size_t i = 0; i < 25; i++)
  {
    EXPECT_EQ(
      controller_interface::return_type::OK,
      cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  }
  EXPECT_GE(test_controller->internal_counter, 1u);
  EXPECT_EQ(test_controller->get_update_rate(), 4u);
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerManagerWithStrictness,
  testing::Values(strict, best_effort));

class TestControllerManagerWithUpdateRates
: public ControllerManagerFixture<controller_manager::ControllerManager>,
  public testing::WithParamInterface<unsigned int>
{
};

TEST_P(TestControllerManagerWithUpdateRates, per_controller_equal_and_higher_update_rate)
{
  const auto strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  const unsigned int ctrl_update_rate = GetParam();
  auto test_controller = std::make_shared<test_controller::TestController>();

  auto last_internal_counter = 0u;
  RCLCPP_INFO(
    rclcpp::get_logger("test_controller_manager"), "Testing update rate : %u Hz", ctrl_update_rate);
  {
    ControllerManagerRunner cm_runner(this);
    cm_->add_controller(
      test_controller, test_controller::TEST_CONTROLLER_NAME,
      test_controller::TEST_CONTROLLER_CLASS_NAME);
  }
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(last_internal_counter, test_controller->internal_counter)
    << "Update should not reach an unconfigured controller";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_state().id());

  rclcpp::Parameter update_rate_parameter("update_rate", static_cast<int>(ctrl_update_rate));
  test_controller->get_node()->set_parameter(update_rate_parameter);
  // configure controller
  {
    ControllerManagerRunner cm_runner(this);
    cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  }
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(last_internal_counter, test_controller->internal_counter)
    << "Controller is not started";
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_state().id());

  // Start controller, will take effect at the end of the update function
  std::vector<std::string> start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  std::vector<std::string> stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  time_ += rclcpp::Duration::from_seconds(0.01);
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(last_internal_counter, test_controller->internal_counter)
    << "Controller is started at the end of update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_lifecycle_state().id());

  const auto pre_internal_counter = test_controller->internal_counter;
  rclcpp::Rate loop_rate(cm_->get_update_rate());
  const auto cm_update_rate = cm_->get_update_rate();
  for (size_t i = 0; i < 2 * cm_->get_update_rate(); i++)
  {
    time_ += rclcpp::Duration::from_seconds(0.01);
    EXPECT_EQ(
      controller_interface::return_type::OK,
      cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
    // In case of a non perfect divisor, the update period should respect the rule
    // [cm_update_rate, 2*cm_update_rate)
    EXPECT_THAT(
      test_controller->update_period_,
      testing::AllOf(
        testing::Ge(rclcpp::Duration::from_seconds(1.0 / cm_update_rate)),
        testing::Lt(rclcpp::Duration::from_seconds(2.0 / cm_update_rate))));
    loop_rate.sleep();
  }
  // if we do 2 times of the controller_manager update rate, the internal counter should be
  // similarly incremented
  EXPECT_EQ(test_controller->internal_counter, pre_internal_counter + (2 * cm_->get_update_rate()));
  EXPECT_EQ(test_controller->get_update_rate(), ctrl_update_rate);

  auto deactivate_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    stop_controllers, start_controllers, strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, deactivate_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  EXPECT_EQ(
    controller_interface::return_type::OK, cm_->update(time_, rclcpp::Duration::from_seconds(0.01)))
    << "Controller is stopped at the end of update, so it should have done one more update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, deactivate_future.get());
  }
  auto unload_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::unload_controller, cm_,
    test_controller::TEST_CONTROLLER_NAME);
  ASSERT_EQ(std::future_status::timeout, unload_future.wait_for(std::chrono::milliseconds(100)))
    << "unload_controller should be blocking until next update cycle";
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(controller_interface::return_type::OK, unload_future.get());
  last_internal_counter = test_controller->internal_counter;
}

INSTANTIATE_TEST_SUITE_P(
  per_controller_equal_and_higher_update_rate, TestControllerManagerWithUpdateRates,
  testing::Values(100, 232, 400));

class TestControllerUpdateRates
: public ControllerManagerFixture<controller_manager::ControllerManager>,
  public testing::WithParamInterface<unsigned int>
{
};

TEST_P(TestControllerUpdateRates, check_the_controller_update_rate)
{
  const unsigned int ctrl_update_rate = GetParam();
  auto test_controller = std::make_shared<test_controller::TestController>();
  cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(2, test_controller.use_count());

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    test_controller->get_lifecycle_state().id());

  test_controller->get_node()->set_parameter({"update_rate", static_cast<int>(ctrl_update_rate)});
  // configure controller
  {
    ControllerManagerRunner cm_runner(this);
    cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  }
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    test_controller->get_lifecycle_state().id());

  // Start controller, will take effect at the end of the update function
  time_ = test_controller->get_node()->now();  // set to something nonzero
  const auto strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  std::vector<std::string> stop_controllers = {};
  auto switch_future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
    start_controllers, stop_controllers, strictness, true, rclcpp::Duration(0, 0));

  ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
    << "switch_controller should be blocking until next update cycle";

  time_ += rclcpp::Duration::from_seconds(0.01);
  EXPECT_EQ(
    controller_interface::return_type::OK,
    cm_->update(time_, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is started at the end of update";
  {
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_lifecycle_state().id());

  EXPECT_EQ(test_controller->get_update_rate(), ctrl_update_rate);
  const auto cm_update_rate = cm_->get_update_rate();
  const auto controller_update_rate = test_controller->get_update_rate();

  const auto initial_counter = test_controller->internal_counter;
  // don't start with zero to check if the period is correct if controller is activated anytime
  rclcpp::Time time = time_;
  for (size_t update_counter = 0; update_counter <= 10 * cm_update_rate; ++update_counter)
  {
    EXPECT_EQ(
      controller_interface::return_type::OK,
      cm_->update(time, rclcpp::Duration::from_seconds(0.01)));
    // In case of a non perfect divisor, the update period should respect the rule
    // [controller_update_rate, 2*controller_update_rate)
    EXPECT_THAT(
      test_controller->update_period_,
      testing::AllOf(
        testing::Ge(rclcpp::Duration::from_seconds(1.0 / controller_update_rate)),
        testing::Lt(rclcpp::Duration::from_seconds(2.0 / controller_update_rate))))
      << "update_counter: " << update_counter;

    time += rclcpp::Duration::from_seconds(0.01);
    if (update_counter % cm_update_rate == 0)
    {
      const double no_of_secs_passed = static_cast<double>(update_counter) / cm_update_rate;
      // NOTE: here EXPECT_NEAR is used because it is observed that in the first iteration of whole
      // cycle of cm_update_rate counts, there is one count missing, but in rest of the 9 cycles it
      // is clearly tracking, so adding 1 here won't affect the final count.
      // For instance, a controller with update rate 37 Hz, seems to have 36 in the first update
      // cycle and then on accumulating 37 on every other update cycle so at the end of the 10
      // cycles it will have 369 instead of 370.
      EXPECT_THAT(
        test_controller->internal_counter - initial_counter,
        testing::AnyOf(
          testing::Eq(controller_update_rate * no_of_secs_passed),
          testing::Eq((controller_update_rate * no_of_secs_passed) - 1)));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  per_controller_update_rate_check, TestControllerUpdateRates,
  testing::Values(10, 12, 16, 23, 37, 40, 50, 63, 71, 85, 98));
