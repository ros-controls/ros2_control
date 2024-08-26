// Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;

using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_STATE_INTERFACES;

class TestControllerManagerWithTestableCM;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerManagerWithTestableCM;

  FRIEND_TEST(TestControllerManagerWithTestableCM, check_cached_controllers_for_hardware);
  FRIEND_TEST(TestControllerManagerWithTestableCM, stop_controllers_on_hardware_read_error);
  FRIEND_TEST(TestControllerManagerWithTestableCM, stop_controllers_on_hardware_write_error);
  FRIEND_TEST(TestControllerManagerWithTestableCM, stop_controllers_on_multiple_hardware_error);

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
  void SetupAndConfigureControllers(int strictness)
  {
    test_controller_actuator = std::make_shared<test_controller::TestController>();
    cm_->add_controller(
      test_controller_actuator, TEST_CONTROLLER_ACTUATOR_NAME,
      test_controller::TEST_CONTROLLER_CLASS_NAME);
    controller_interface::InterfaceConfiguration test_controller_actuator_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES};
    controller_interface::InterfaceConfiguration test_controller_actuator_state_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES};
    test_controller_actuator->set_command_interface_configuration(
      test_controller_actuator_cmd_ifs_cfg);
    test_controller_actuator->set_state_interface_configuration(
      test_controller_actuator_state_ifs_cfg);

    test_controller_system = std::make_shared<test_controller::TestController>();
    cm_->add_controller(
      test_controller_system, TEST_CONTROLLER_SYSTEM_NAME,
      test_controller::TEST_CONTROLLER_CLASS_NAME);
    controller_interface::InterfaceConfiguration test_system_controller_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES};
    controller_interface::InterfaceConfiguration test_system_controller_state_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES};
    test_controller_system->set_command_interface_configuration(test_system_controller_cmd_ifs_cfg);
    test_controller_system->set_state_interface_configuration(test_system_controller_state_ifs_cfg);

    test_broadcaster_all = std::make_shared<test_controller::TestController>();
    cm_->add_controller(
      test_broadcaster_all, TEST_BROADCASTER_ALL_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME);
    controller_interface::InterfaceConfiguration test_broadcaster_all_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::NONE, {}};
    controller_interface::InterfaceConfiguration test_broadcaster_all_state_ifs_cfg = {
      controller_interface::interface_configuration_type::ALL, {}};
    test_broadcaster_all->set_command_interface_configuration(test_broadcaster_all_cmd_ifs_cfg);
    test_broadcaster_all->set_state_interface_configuration(test_broadcaster_all_state_ifs_cfg);

    test_broadcaster_sensor = std::make_shared<test_controller::TestController>();
    cm_->add_controller(
      test_broadcaster_sensor, TEST_BROADCASTER_SENSOR_NAME,
      test_controller::TEST_CONTROLLER_CLASS_NAME);
    controller_interface::InterfaceConfiguration test_broadcaster_sensor_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::NONE, {}};
    controller_interface::InterfaceConfiguration test_broadcaster_sensor_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      TEST_SENSOR_HARDWARE_STATE_INTERFACES};
    test_broadcaster_sensor->set_command_interface_configuration(
      test_broadcaster_sensor_cmd_ifs_cfg);
    test_broadcaster_sensor->set_state_interface_configuration(test_broadcaster_sensor_ifs_cfg);

    // check if all controllers are added correctly
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      test_broadcaster_sensor->get_lifecycle_state().id());

    // configure controllers
    cm_->configure_controller(TEST_CONTROLLER_ACTUATOR_NAME);
    cm_->configure_controller(TEST_CONTROLLER_SYSTEM_NAME);
    cm_->configure_controller(TEST_BROADCASTER_ALL_NAME);
    cm_->configure_controller(TEST_BROADCASTER_SENSOR_NAME);

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(0u, test_controller_actuator->internal_counter) << "Controller is not started";
    EXPECT_EQ(0u, test_controller_system->internal_counter) << "Controller is not started";
    EXPECT_EQ(0u, test_broadcaster_all->internal_counter) << "Controller is not started";
    EXPECT_EQ(0u, test_broadcaster_sensor->internal_counter) << "Controller is not started";

    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    // Start controller, will take effect at the end of the update function
    switch_test_controllers(
      {TEST_CONTROLLER_ACTUATOR_NAME, TEST_CONTROLLER_SYSTEM_NAME, TEST_BROADCASTER_ALL_NAME,
       TEST_BROADCASTER_SENSOR_NAME},
      {}, strictness);
  }

  static constexpr char TEST_CONTROLLER_ACTUATOR_NAME[] = "test_controller_actuator";
  static constexpr char TEST_CONTROLLER_SYSTEM_NAME[] = "test_controller_system";
  static constexpr char TEST_BROADCASTER_ALL_NAME[] = "test_broadcaster_all";
  static constexpr char TEST_BROADCASTER_SENSOR_NAME[] = "test_broadcaster_sensor";

  std::shared_ptr<test_controller::TestController> test_controller_actuator;
  std::shared_ptr<test_controller::TestController> test_controller_system;
  std::shared_ptr<test_controller::TestController> test_broadcaster_all;
  std::shared_ptr<test_controller::TestController> test_broadcaster_sensor;
};

TEST_P(TestControllerManagerWithTestableCM, check_cached_controllers_for_hardware)
{
  auto strictness = GetParam().strictness;
  SetupAndConfigureControllers(strictness);

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_actuator->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_system->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_all->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_sensor->get_lifecycle_state().id());

  {
    auto controllers =
      cm_->resource_manager_->get_cached_controllers_to_hardware(TEST_ACTUATOR_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::UnorderedElementsAreArray(std::vector<std::string>(
                     {TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME})));
  }

  {
    auto controllers =
      cm_->resource_manager_->get_cached_controllers_to_hardware(TEST_SYSTEM_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::UnorderedElementsAreArray(std::vector<std::string>(
                     {TEST_CONTROLLER_SYSTEM_NAME, TEST_BROADCASTER_ALL_NAME})));
  }

  {
    auto controllers =
      cm_->resource_manager_->get_cached_controllers_to_hardware(TEST_SENSOR_HARDWARE_NAME);
    ASSERT_THAT(
      controllers, testing::UnorderedElementsAreArray(std::vector<std::string>(
                     {TEST_BROADCASTER_SENSOR_NAME, TEST_BROADCASTER_ALL_NAME})));
  }
}

TEST_P(TestControllerManagerWithTestableCM, stop_controllers_on_hardware_read_error)
{
  auto strictness = GetParam().strictness;
  SetupAndConfigureControllers(strictness);

  rclcpp_lifecycle::State state_active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  {
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_GE(test_controller_actuator->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_controller_system->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_all->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_sensor->internal_counter, 1u)
      << "Controller is started at the end of update";
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_actuator->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_system->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_all->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_sensor->get_lifecycle_state().id());

  // Execute first time without any errors
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter) << "Execute without errors";
  }

  // Simulate error in read() on TEST_ACTUATOR_HARDWARE_NAME by setting first command interface to
  // READ_FAIL_VALUE
  test_controller_actuator->set_first_command_interface_value_to = test_constants::READ_FAIL_VALUE;
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  {
    auto previous_counter = test_controller_actuator->internal_counter;
    auto new_counter = test_controller_system->internal_counter + 1;

    // here happens error in hardware and
    // "actuator controller" and "broadcaster all" are deactivated
    EXPECT_NO_THROW(cm_->read(time_, PERIOD));
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter)
      << "Execute has read error and it is not updated";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter)
      << "Broadcaster for all interfaces is not updated";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  // Recover hardware and activate again all controllers
  {
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    auto status_map = cm_->resource_manager_->get_components_status();
    ASSERT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter_higher = test_controller_system->internal_counter;

    switch_test_controllers(
      {TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME}, {}, strictness);

    EXPECT_GT(test_controller_actuator->internal_counter, previous_counter_lower);
    EXPECT_LE(test_controller_actuator->internal_counter, previous_counter_higher);
    EXPECT_GT(test_controller_system->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_all->internal_counter, previous_counter_lower);
    EXPECT_LE(test_broadcaster_all->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_sensor->internal_counter, previous_counter_higher);
  }

  // Simulate error in read() on TEST_ACTUATOR_HARDWARE_NAME and TEST_SYSTEM_HARDWARE_NAME
  // by setting first command interface to READ_FAIL_VALUE
  test_controller_actuator->set_first_command_interface_value_to = test_constants::READ_FAIL_VALUE;
  test_controller_system->set_first_command_interface_value_to = test_constants::READ_FAIL_VALUE;
  {
    auto previous_counter_lower = test_controller_actuator->internal_counter + 1;
    auto previous_counter_higher = test_controller_system->internal_counter + 1;

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));

    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter_lower)
      << "Execute without errors to write value";
    EXPECT_EQ(test_controller_system->internal_counter, previous_counter_higher)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter_lower)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, previous_counter_higher)
      << "Execute without errors to write value";
  }

  {
    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter_higher = test_controller_system->internal_counter;
    auto new_counter = test_broadcaster_sensor->internal_counter + 1;

    EXPECT_NO_THROW(cm_->read(time_, PERIOD));
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter_lower)
      << "Execute has read error and it is not updated";
    EXPECT_EQ(test_controller_system->internal_counter, previous_counter_higher)
      << "Execute has read error and it is not updated";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter_lower)
      << "Broadcaster for all interfaces is not updated";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  // Recover hardware and activate again all controllers
  {
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    auto status_map = cm_->resource_manager_->get_components_status();
    ASSERT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter = test_controller_system->internal_counter;
    auto previous_counter_higher = test_broadcaster_sensor->internal_counter;

    switch_test_controllers(
      {TEST_CONTROLLER_SYSTEM_NAME, TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME}, {},
      strictness);

    EXPECT_GT(test_controller_actuator->internal_counter, previous_counter_lower);
    EXPECT_LE(test_controller_actuator->internal_counter, previous_counter_higher);
    EXPECT_GT(test_controller_system->internal_counter, previous_counter);
    EXPECT_LE(test_controller_system->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_all->internal_counter, previous_counter_lower);
    EXPECT_LE(test_broadcaster_all->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_sensor->internal_counter, previous_counter_higher);
  }
}

TEST_P(TestControllerManagerWithTestableCM, stop_controllers_on_controller_error)
{
  auto strictness = GetParam().strictness;
  SetupAndConfigureControllers(strictness);

  rclcpp_lifecycle::State state_active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  {
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_GE(test_controller_actuator->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_controller_system->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_all->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_sensor->internal_counter, 1u)
      << "Controller is started at the end of update";
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_actuator->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_system->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_all->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_sensor->get_lifecycle_state().id());

  // Execute first time without any errors
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter) << "Execute without errors";
  }

  // Simulate error in update method of the controllers but not in hardware
  test_controller_actuator->external_commands_for_testing_[0] =
    std::numeric_limits<double>::quiet_NaN();
  test_controller_system->external_commands_for_testing_[0] =
    std::numeric_limits<double>::quiet_NaN();
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::ERROR, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter)
      << "Executes the current cycle and returns ERROR";
    EXPECT_EQ(
      test_controller_actuator->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(test_controller_system->internal_counter, new_counter)
      << "Executes the current cycle and returns ERROR";
    EXPECT_EQ(
      test_controller_system->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  {
    auto previous_counter = test_controller_actuator->internal_counter;
    auto new_counter = test_controller_system->internal_counter + 1;

    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter)
      << "Cannot execute as it should be currently deactivated";
    EXPECT_EQ(test_controller_system->internal_counter, previous_counter)
      << "Cannot execute as it should be currently deactivated";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter)
      << "Broadcaster all interfaces without errors";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";

    // The states shouldn't change as there are no more controller errors
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());
  }
}

TEST_P(TestControllerManagerWithTestableCM, stop_controllers_on_hardware_write_error)
{
  auto strictness = GetParam().strictness;
  SetupAndConfigureControllers(strictness);

  rclcpp_lifecycle::State state_active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  {
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_GE(test_controller_actuator->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_controller_system->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_all->internal_counter, 1u)
      << "Controller is started at the end of update";
    EXPECT_GE(test_broadcaster_sensor->internal_counter, 1u)
      << "Controller is started at the end of update";
  }

  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_actuator->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_controller_system->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_all->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    test_broadcaster_sensor->get_lifecycle_state().id());

  // Execute first time without any errors
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter) << "Execute without errors";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter) << "Execute without errors";
  }

  // Simulate error in write() on TEST_ACTUATOR_HARDWARE_NAME by setting first command interface to
  // WRITE_FAIL_VALUE
  test_controller_actuator->set_first_command_interface_value_to = test_constants::WRITE_FAIL_VALUE;
  {
    auto new_counter = test_controller_actuator->internal_counter + 1;
    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  {
    auto previous_counter = test_controller_actuator->internal_counter;
    auto new_counter = test_controller_system->internal_counter + 1;

    // here happens error in hardware and
    // "actuator controller" and "broadcaster all" are deactivated
    EXPECT_NO_THROW(cm_->write(time_, PERIOD));
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_controller_system->internal_counter, new_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  // Recover hardware and activate again all controllers
  {
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    auto status_map = cm_->resource_manager_->get_components_status();
    ASSERT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter_higher = test_controller_system->internal_counter;

    switch_test_controllers(
      {TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME}, {}, strictness);

    EXPECT_GT(test_controller_actuator->internal_counter, previous_counter_lower);
    EXPECT_LE(test_controller_actuator->internal_counter, previous_counter_higher);
    EXPECT_GT(test_controller_system->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_all->internal_counter, previous_counter_lower);
    EXPECT_LE(test_broadcaster_all->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_sensor->internal_counter, previous_counter_higher);
  }

  // Simulate error in write() on TEST_ACTUATOR_HARDWARE_NAME and TEST_SYSTEM_HARDWARE_NAME
  // by setting first command interface to WRITE_FAIL_VALUE
  test_controller_actuator->set_first_command_interface_value_to = test_constants::WRITE_FAIL_VALUE;
  test_controller_system->set_first_command_interface_value_to = test_constants::WRITE_FAIL_VALUE;
  {
    auto previous_counter_lower = test_controller_actuator->internal_counter + 1;
    auto previous_counter_higher = test_controller_system->internal_counter + 1;

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));

    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter_lower)
      << "Execute without errors to write value";
    EXPECT_EQ(test_controller_system->internal_counter, previous_counter_higher)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter_lower)
      << "Execute without errors to write value";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, previous_counter_higher)
      << "Execute without errors to write value";
  }

  {
    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter_higher = test_controller_system->internal_counter;
    auto new_counter = test_broadcaster_sensor->internal_counter + 1;

    // here happens error in hardware and
    // "actuator controller" and "broadcaster all" are deactivated
    EXPECT_NO_THROW(cm_->write(time_, PERIOD));
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_actuator->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_controller_system->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      test_broadcaster_all->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      test_broadcaster_sensor->get_lifecycle_state().id());

    EXPECT_EQ(controller_interface::return_type::OK, cm_->update(time_, PERIOD));
    EXPECT_EQ(test_controller_actuator->internal_counter, previous_counter_lower)
      << "Execute has write error and it is not updated";
    EXPECT_EQ(test_controller_system->internal_counter, previous_counter_higher)
      << "Execute has write error and it is not updated";
    EXPECT_EQ(test_broadcaster_all->internal_counter, previous_counter_lower)
      << "Broadcaster for all interfaces is not updated";
    EXPECT_EQ(test_broadcaster_sensor->internal_counter, new_counter)
      << "Execute without errors to write value";
  }

  // Recover hardware and activate again all controllers
  {
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    ASSERT_EQ(
      cm_->resource_manager_->set_component_state(
        ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME, state_active),
      hardware_interface::return_type::OK);
    auto status_map = cm_->resource_manager_->get_components_status();
    ASSERT_EQ(
      status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(
      status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto previous_counter_lower = test_controller_actuator->internal_counter;
    auto previous_counter = test_controller_system->internal_counter;
    auto previous_counter_higher = test_broadcaster_sensor->internal_counter;

    switch_test_controllers(
      {TEST_CONTROLLER_SYSTEM_NAME, TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME}, {},
      strictness);

    EXPECT_GT(test_controller_actuator->internal_counter, previous_counter_lower);
    EXPECT_LE(test_controller_actuator->internal_counter, previous_counter_higher);
    EXPECT_GT(test_controller_system->internal_counter, previous_counter);
    EXPECT_LE(test_controller_system->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_all->internal_counter, previous_counter_lower);
    EXPECT_LE(test_broadcaster_all->internal_counter, previous_counter_higher);
    EXPECT_GT(test_broadcaster_sensor->internal_counter, previous_counter_higher);
  }
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerManagerWithTestableCM,
  testing::Values(strict, best_effort));
