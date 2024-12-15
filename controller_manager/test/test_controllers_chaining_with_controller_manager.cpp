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
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_chainable_controller/test_chainable_controller.hpp"
#include "test_controller/test_controller.hpp"

// The tests in this file are implementing example of chained-control for DiffDrive robot example:
// https://github.com/ros-controls/roadmap/blob/9f32e215a84347aee0b519cb24d081f23bbbb224/design_drafts/cascade_control.md#motivation-purpose-and-use
// The controller have the names as stated in figure, but they are simply forwarding values without
// functionality that their name would suggest

class TestControllerChainingWithControllerManager;

class TestableTestChainableController : public test_chainable_controller::TestChainableController
{
  friend TestControllerChainingWithControllerManager;

  FRIEND_TEST(TestControllerChainingWithControllerManager, test_chained_controllers);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_auto_switch_to_chained_mode);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_activation_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_activation_switching_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_deactivation_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager, test_chained_controllers_adding_in_random_order);
};

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerChainingWithControllerManager;

  FRIEND_TEST(
    TestControllerChainingWithControllerManagerAndChainedControllersParameter,
    test_cm_reading_chained_controllers_parameter);
  FRIEND_TEST(
    TestControllerChainingWithControllerManagerAndChainedControllersParameter,
    test_cm_reading_chained_controllers_parameter_failure_group0);
  FRIEND_TEST(
    TestControllerChainingWithControllerManagerAndChainedControllersParameter,
    test_cm_reading_chained_controllers_parameter_failure_wrong_type);
  FRIEND_TEST(
    TestControllerChainingWithControllerManagerAndChainedControllersParameter,
    test_cm_reading_chained_controllers_parameter_failure_duplicated_controller);

  FRIEND_TEST(TestControllerChainingWithControllerManager, test_chained_controllers);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_auto_switch_to_chained_mode);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_activation_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_activation_switching_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager,
    test_chained_controllers_deactivation_error_handling);
  FRIEND_TEST(
    TestControllerChainingWithControllerManager, test_chained_controllers_adding_in_random_order);

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
        ros2_control_test_assets::diffbot_urdf, rm_node_->get_node_clock_interface(),
        rm_node_->get_node_logging_interface(), true),
      executor_, TEST_CM_NAME);
    run_updater_ = false;
  }

  void SetupControllers()
  {
    test_param = GetParam();

    pid_left_wheel_controller = std::make_shared<TestableTestChainableController>();
    pid_right_wheel_controller = std::make_shared<TestableTestChainableController>();
    diff_drive_controller = std::make_shared<TestableTestChainableController>();
    diff_drive_controller_two = std::make_shared<TestableTestChainableController>();
    position_tracking_controller = std::make_shared<test_controller::TestController>();
    position_tracking_controller_two = std::make_shared<test_controller::TestController>();
    odom_publisher_controller = std::make_shared<test_controller::TestController>();
    sensor_fusion_controller = std::make_shared<TestableTestChainableController>();
    robot_localization_controller = std::make_shared<TestableTestChainableController>();

    // configure Left Wheel controller
    controller_interface::InterfaceConfiguration pid_left_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}};
    controller_interface::InterfaceConfiguration pid_left_state_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}};
    pid_left_wheel_controller->set_command_interface_configuration(pid_left_cmd_ifs_cfg);
    pid_left_wheel_controller->set_state_interface_configuration(pid_left_state_ifs_cfg);
    pid_left_wheel_controller->set_reference_interface_names({"velocity"});

    // configure Right Wheel controller
    controller_interface::InterfaceConfiguration pid_right_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}};
    controller_interface::InterfaceConfiguration pid_right_state_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}};
    pid_right_wheel_controller->set_command_interface_configuration(pid_right_cmd_ifs_cfg);
    pid_right_wheel_controller->set_state_interface_configuration(pid_right_state_ifs_cfg);
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
    diff_drive_controller->set_exported_state_interface_names({"odom_x", "odom_y"});

    // configure Diff Drive Two controller (Has same command interfaces as Diff Drive controller)
    diff_drive_controller_two->set_command_interface_configuration(diff_drive_cmd_ifs_cfg);
    diff_drive_controller_two->set_state_interface_configuration(diff_drive_state_ifs_cfg);
    diff_drive_controller_two->set_reference_interface_names({"vel_x", "vel_y", "rot_z"});
    diff_drive_controller_two->set_exported_state_interface_names({"odom_x", "odom_y"});

    // configure Position Tracking controller
    controller_interface::InterfaceConfiguration position_tracking_cmd_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {std::string(DIFF_DRIVE_CONTROLLER) + "/vel_x",
       std::string(DIFF_DRIVE_CONTROLLER) + "/vel_y"}};
    // in this simple example "vel_x" == "velocity left wheel" and "vel_y" == "velocity right wheel"
    position_tracking_controller->set_command_interface_configuration(
      position_tracking_cmd_ifs_cfg);

    // configure Odometry Publisher controller
    controller_interface::InterfaceConfiguration odom_and_fusion_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {std::string(DIFF_DRIVE_CONTROLLER) + "/odom_x",
       std::string(DIFF_DRIVE_CONTROLLER) + "/odom_y"}};
    odom_publisher_controller->set_state_interface_configuration(odom_and_fusion_ifs_cfg);

    // configure sensor fusion controller
    sensor_fusion_controller->set_imu_sensor_name("base_imu");
    sensor_fusion_controller->set_state_interface_configuration(odom_and_fusion_ifs_cfg);
    sensor_fusion_controller->set_exported_state_interface_names({"odom_x", "odom_y", "yaw"});

    // configure Robot Localization controller
    controller_interface::InterfaceConfiguration odom_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {std::string(SENSOR_FUSION_CONTROLLER) + "/odom_x",
       std::string(SENSOR_FUSION_CONTROLLER) + "/odom_y",
       std::string(SENSOR_FUSION_CONTROLLER) + "/yaw"}};
    robot_localization_controller->set_state_interface_configuration(odom_ifs_cfg);
    robot_localization_controller->set_exported_state_interface_names({"actual_pose"});

    // configure Position Tracking controller Two
    controller_interface::InterfaceConfiguration position_tracking_state_ifs_cfg = {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {std::string(ROBOT_LOCALIZATION_CONTROLLER) + "/actual_pose"}};
    // in this simple example "vel_x" == "velocity left wheel" and "vel_y" == "velocity right wheel"
    position_tracking_controller_two->set_command_interface_configuration(
      position_tracking_cmd_ifs_cfg);
    position_tracking_controller_two->set_state_interface_configuration(
      position_tracking_state_ifs_cfg);
  }

  void CheckIfControllersAreAddedCorrectly()
  {
    EXPECT_EQ(9u, cm_->get_loaded_controllers().size());
    EXPECT_EQ(2, pid_left_wheel_controller.use_count());
    EXPECT_EQ(2, pid_right_wheel_controller.use_count());
    EXPECT_EQ(2, diff_drive_controller.use_count());
    EXPECT_EQ(2, diff_drive_controller_two.use_count());
    EXPECT_EQ(2, position_tracking_controller.use_count());
    EXPECT_EQ(2, sensor_fusion_controller.use_count());
    EXPECT_EQ(2, robot_localization_controller.use_count());
    EXPECT_EQ(2, odom_publisher_controller.use_count());
    EXPECT_EQ(2, position_tracking_controller_two.use_count());

    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      pid_left_wheel_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      pid_right_wheel_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      diff_drive_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      diff_drive_controller_two->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      position_tracking_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      sensor_fusion_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      robot_localization_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      odom_publisher_controller->get_lifecycle_state().id());
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      position_tracking_controller_two->get_lifecycle_state().id());
  }

  // order or controller configuration is not important therefore we can reuse the same method
  void ConfigureAndCheckControllers()
  {
    // Store initial values of command interfaces
    size_t number_of_cmd_itfs = cm_->resource_manager_->command_interface_keys().size();
    size_t number_of_state_itfs = cm_->resource_manager_->state_interface_keys().size();

    // configure chainable controller and check exported interfaces
    cm_->configure_controller(PID_LEFT_WHEEL);
    EXPECT_EQ(
      pid_left_wheel_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 1);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs);
    for (const auto & interface : {"pid_left_wheel_controller/velocity"})
    {
      EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
    }

    cm_->configure_controller(PID_RIGHT_WHEEL);
    EXPECT_EQ(
      pid_right_wheel_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 2);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs);
    for (const auto & interface : {"pid_right_wheel_controller/velocity"})
    {
      EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
    }

    cm_->configure_controller(DIFF_DRIVE_CONTROLLER);
    for (auto & x : cm_->resource_manager_->available_state_interfaces())
    {
      RCLCPP_ERROR_STREAM(cm_->get_logger(), x);
    }
    EXPECT_EQ(
      diff_drive_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 5);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 2);
    for (const auto & interface :
         {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y",
          "diff_drive_controller/rot_z"})
    {
      EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
    }
    for (const auto & interface : {"diff_drive_controller/odom_x", "diff_drive_controller/odom_y"})
    {
      EXPECT_TRUE(cm_->resource_manager_->state_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->state_interface_is_available(interface));
    }

    cm_->configure_controller(DIFF_DRIVE_CONTROLLER_TWO);
    EXPECT_EQ(
      diff_drive_controller_two->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 4);
    for (const auto & interface :
         {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y",
          "diff_drive_controller/rot_z"})
    {
      EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface));
      EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
    }
    for (const auto & interface : {"diff_drive_controller/odom_x", "diff_drive_controller/odom_y"})
    {
      EXPECT_TRUE(cm_->resource_manager_->state_interface_exists(interface));
      EXPECT_FALSE(cm_->resource_manager_->state_interface_is_available(interface));
    }

    cm_->configure_controller(POSITION_TRACKING_CONTROLLER);
    EXPECT_EQ(
      position_tracking_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 4);

    cm_->configure_controller(SENSOR_FUSION_CONTROLLER);
    EXPECT_EQ(
      position_tracking_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 7);

    cm_->configure_controller(ROBOT_LOCALIZATION_CONTROLLER);
    EXPECT_EQ(
      position_tracking_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 8);

    cm_->configure_controller(ODOM_PUBLISHER_CONTROLLER);
    EXPECT_EQ(
      position_tracking_controller->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 8);

    cm_->configure_controller(POSITION_TRACKING_CONTROLLER_TWO);
    EXPECT_EQ(
      position_tracking_controller_two->get_lifecycle_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(cm_->resource_manager_->command_interface_keys().size(), number_of_cmd_itfs + 8);
    EXPECT_EQ(cm_->resource_manager_->state_interface_keys().size(), number_of_state_itfs + 8);
  }

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  void SetToChainedModeAndMakeInterfacesAvailable(
    std::shared_ptr<T> & controller, const std::string & controller_name,
    const std::vector<std::string> & exported_state_interfaces,
    const std::vector<std::string> & reference_interfaces)
  {
    if (!exported_state_interfaces.empty() || !reference_interfaces.empty())
    {
      controller->set_chained_mode(true);
      EXPECT_TRUE(controller->is_in_chained_mode());
      if (!reference_interfaces.empty())
      {
        // make reference interface command_interfaces available
        cm_->resource_manager_->make_controller_reference_interfaces_available(controller_name);
        for (const auto & interface : reference_interfaces)
        {
          EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
          EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
          EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
        }
      }
      if (!exported_state_interfaces.empty())
      {
        // make state interface state_interfaces available
        cm_->resource_manager_->make_controller_exported_state_interfaces_available(
          controller_name);
        for (const auto & interface : exported_state_interfaces)
        {
          EXPECT_TRUE(cm_->resource_manager_->state_interface_exists(interface));
          EXPECT_TRUE(cm_->resource_manager_->state_interface_is_available(interface));
        }
      }
    }
  }

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  void check_after_de_activate(
    std::shared_ptr<T> & controller, const std::vector<std::string> & claimed_command_itfs,
    size_t expected_internal_counter, const controller_interface::return_type expected_return,
    bool deactivated, bool claimed_interfaces_from_hw = false)
  {
    for (const auto & interface : claimed_command_itfs)
    {
      EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
      // successful xor deactivated
      if ((expected_return == controller_interface::return_type::OK) != deactivated)
      {
        EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
        EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed(interface));
      }
      else
      {
        if (claimed_interfaces_from_hw)
        {
          EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface))
            << "The interface : " << interface << " should be available but it is not available";
        }
        else
        {
          EXPECT_FALSE(cm_->resource_manager_->command_interface_is_available(interface))
            << "The interface : " << interface << " shouldn't be available but it is available";
        }
        EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
      }
    }
    if (expected_internal_counter != 0)
    {
      ASSERT_EQ(controller->internal_counter, expected_internal_counter);
    }
  }

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  void ActivateAndCheckController(
    std::shared_ptr<T> & controller, const std::string & controller_name,
    const std::vector<std::string> & claimed_command_itfs, size_t expected_internal_counter = 0u,
    const controller_interface::return_type expected_return = controller_interface::return_type::OK,
    const std::future_status expected_future_status = std::future_status::timeout)
  {
    switch_test_controllers(
      {controller_name}, {}, test_param.strictness, expected_future_status, expected_return);
    check_after_de_activate(
      controller, claimed_command_itfs, expected_internal_counter, expected_return, false);
  }

  void ActivateController(
    const std::string & controller_name,
    const controller_interface::return_type expected_return = controller_interface::return_type::OK,
    const std::future_status expected_future_status = std::future_status::timeout)
  {
    switch_test_controllers(
      {controller_name}, {}, test_param.strictness, expected_future_status, expected_return);
  }

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  void DeactivateAndCheckController(
    std::shared_ptr<T> & controller, const std::string & controller_name,
    const std::vector<std::string> & claimed_command_itfs, size_t expected_internal_counter = 0u,
    const bool claimed_interfaces_from_hw = false,
    const controller_interface::return_type expected_return = controller_interface::return_type::OK,
    const std::future_status expected_future_status = std::future_status::timeout)
  {
    switch_test_controllers(
      {}, {controller_name}, test_param.strictness, expected_future_status, expected_return);
    check_after_de_activate(
      controller, claimed_command_itfs, expected_internal_counter, expected_return, true,
      claimed_interfaces_from_hw);
  }

  void DeactivateController(
    const std::string & controller_name,
    const controller_interface::return_type expected_return = controller_interface::return_type::OK,
    const std::future_status expected_future_status = std::future_status::timeout)
  {
    switch_test_controllers(
      {}, {controller_name}, test_param.strictness, expected_future_status, expected_return);
  }

  void UpdateAllControllerAndCheck(
    const std::vector<double> & reference, size_t exp_internal_counter_pos_ctrl)
  {
    // test value that could cause bad-memory access --> cleaner error during writing tests
    ASSERT_EQ(reference.size(), 2u);

    position_tracking_controller->external_commands_for_testing_[0] = reference[0];
    position_tracking_controller->external_commands_for_testing_[1] = reference[1];

    cm_->update(time_, rclcpp::Duration::from_seconds(0.01));
    cm_->resource_manager_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

    // check if all controllers are updated
    ASSERT_EQ(odom_publisher_controller->internal_counter, exp_internal_counter_pos_ctrl);
    ASSERT_EQ(robot_localization_controller->internal_counter, exp_internal_counter_pos_ctrl + 2);
    ASSERT_EQ(sensor_fusion_controller->internal_counter, exp_internal_counter_pos_ctrl + 4);
    ASSERT_EQ(position_tracking_controller->internal_counter, exp_internal_counter_pos_ctrl + 6);
    ASSERT_EQ(diff_drive_controller->internal_counter, exp_internal_counter_pos_ctrl + 8);
    ASSERT_EQ(pid_left_wheel_controller->internal_counter, exp_internal_counter_pos_ctrl + 12);
    ASSERT_EQ(pid_right_wheel_controller->internal_counter, exp_internal_counter_pos_ctrl + 10);

    // check if values are set properly in controllers
    ASSERT_EQ(
      diff_drive_controller->reference_interfaces_[0], reference[0]);  // command from Position to
    ASSERT_EQ(
      diff_drive_controller->reference_interfaces_[1], reference[1]);  // DiffDrive is forwarded

    // Command of DiffDrive controller are references of PID controllers
    EXP_LEFT_WHEEL_REF = chained_ctrl_calculation(reference[0], EXP_LEFT_WHEEL_HW_STATE);
    EXP_RIGHT_WHEEL_REF = chained_ctrl_calculation(reference[1], EXP_RIGHT_WHEEL_HW_STATE);
    ASSERT_EQ(diff_drive_controller->command_interfaces_[0].get_value(), EXP_LEFT_WHEEL_REF);
    ASSERT_EQ(diff_drive_controller->command_interfaces_[1].get_value(), EXP_RIGHT_WHEEL_REF);
    ASSERT_EQ(pid_left_wheel_controller->reference_interfaces_[0], EXP_LEFT_WHEEL_REF);
    ASSERT_EQ(pid_right_wheel_controller->reference_interfaces_[0], EXP_RIGHT_WHEEL_REF);

    EXP_STATE_ODOM_X = chained_estimate_calculation(reference[0], EXP_LEFT_WHEEL_HW_STATE);
    EXP_STATE_ODOM_Y = chained_estimate_calculation(reference[1], EXP_RIGHT_WHEEL_HW_STATE);
    ASSERT_EQ(sensor_fusion_controller->state_interfaces_values_.size(), 3u);
    ASSERT_EQ(robot_localization_controller->get_state_interface_data().size(), 3u);
    ASSERT_EQ(robot_localization_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
    ASSERT_EQ(robot_localization_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);
    ASSERT_EQ(odom_publisher_controller->get_state_interface_data().size(), 2u);
    ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
    ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);

    EXP_LEFT_WHEEL_CMD = chained_ctrl_calculation(EXP_LEFT_WHEEL_REF, EXP_LEFT_WHEEL_HW_STATE);
    EXP_LEFT_WHEEL_HW_STATE = hardware_calculation(EXP_LEFT_WHEEL_CMD);
    ASSERT_EQ(pid_left_wheel_controller->command_interfaces_[0].get_value(), EXP_LEFT_WHEEL_CMD);
    ASSERT_EQ(pid_left_wheel_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);
    // DiffDrive uses the same state
    ASSERT_EQ(diff_drive_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);
    // The state doesn't change wrt to any data from the hardware calculation
    ASSERT_EQ(robot_localization_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
    ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);

    EXP_RIGHT_WHEEL_CMD = chained_ctrl_calculation(EXP_RIGHT_WHEEL_REF, EXP_RIGHT_WHEEL_HW_STATE);
    EXP_RIGHT_WHEEL_HW_STATE = hardware_calculation(EXP_RIGHT_WHEEL_CMD);
    ASSERT_EQ(pid_right_wheel_controller->command_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_CMD);
    ASSERT_EQ(
      pid_right_wheel_controller->state_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_HW_STATE);
    // DiffDrive uses the same state
    ASSERT_EQ(diff_drive_controller->state_interfaces_[1].get_value(), EXP_RIGHT_WHEEL_HW_STATE);
    // The state doesn't change wrt to any data from the hardware calculation
    ASSERT_EQ(robot_localization_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);
    ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);
  }

  // check data propagation through controllers and if individual controllers are working
  double chained_ctrl_calculation(double reference, double state) { return reference - state; }
  double chained_estimate_calculation(double reference, double state)
  {
    return (reference - state) * test_chainable_controller::CONTROLLER_DT;
  }
  double hardware_calculation(double command) { return command / 2.0; }

  // set controllers' names
  static constexpr char PID_LEFT_WHEEL[] = "pid_left_wheel_controller";
  static constexpr char PID_RIGHT_WHEEL[] = "pid_right_wheel_controller";
  static constexpr char DIFF_DRIVE_CONTROLLER[] = "diff_drive_controller";
  static constexpr char DIFF_DRIVE_CONTROLLER_TWO[] = "diff_drive_controller_two";
  static constexpr char POSITION_TRACKING_CONTROLLER[] = "position_tracking_controller";
  static constexpr char SENSOR_FUSION_CONTROLLER[] = "sensor_fusion_controller";
  static constexpr char ROBOT_LOCALIZATION_CONTROLLER[] = "robot_localization_controller";
  static constexpr char ODOM_PUBLISHER_CONTROLLER[] = "odometry_publisher_controller";
  static constexpr char POSITION_TRACKING_CONTROLLER_TWO[] = "position_tracking_controller_two";

  const std::vector<std::string> PID_LEFT_WHEEL_REFERENCE_INTERFACES = {
    "pid_left_wheel_controller/velocity"};
  const std::vector<std::string> PID_RIGHT_WHEEL_REFERENCE_INTERFACES = {
    "pid_right_wheel_controller/velocity"};
  const std::vector<std::string> DIFF_DRIVE_REFERENCE_INTERFACES = {
    "diff_drive_controller/vel_x", "diff_drive_controller/vel_y", "diff_drive_controller/rot_z"};
  const std::vector<std::string> DIFF_DRIVE_STATE_INTERFACES = {
    "diff_drive_controller/odom_x", "diff_drive_controller/odom_y"};
  const std::vector<std::string> SENSOR_FUSION_ESIMTATED_INTERFACES = {
    "sensor_fusion_controller/odom_x", "sensor_fusion_controller/odom_y",
    "sensor_fusion_controller/yaw"};

  const std::vector<std::string> PID_LEFT_WHEEL_CLAIMED_INTERFACES = {"wheel_left/velocity"};
  const std::vector<std::string> PID_RIGHT_WHEEL_CLAIMED_INTERFACES = {"wheel_right/velocity"};
  const std::vector<std::string> DIFF_DRIVE_CLAIMED_INTERFACES = {
    "pid_left_wheel_controller/velocity", "pid_right_wheel_controller/velocity"};
  const std::vector<std::string> POSITION_CONTROLLER_CLAIMED_INTERFACES = {
    "diff_drive_controller/vel_x", "diff_drive_controller/vel_y"};

  // controllers objects
  std::shared_ptr<TestableTestChainableController> pid_left_wheel_controller;
  std::shared_ptr<TestableTestChainableController> pid_right_wheel_controller;
  std::shared_ptr<TestableTestChainableController> diff_drive_controller;
  std::shared_ptr<TestableTestChainableController> diff_drive_controller_two;
  std::shared_ptr<TestableTestChainableController> sensor_fusion_controller;
  std::shared_ptr<TestableTestChainableController> robot_localization_controller;
  std::shared_ptr<test_controller::TestController> odom_publisher_controller;
  std::shared_ptr<test_controller::TestController> position_tracking_controller;
  std::shared_ptr<test_controller::TestController> position_tracking_controller_two;

  testing::WithParamInterface<Strictness>::ParamType test_param;

  // expected values for tests - shared between multiple test runs
  double EXP_LEFT_WHEEL_CMD = 0.0;
  double EXP_LEFT_WHEEL_HW_STATE = 0.0;
  double EXP_RIGHT_WHEEL_CMD = 0.0;
  double EXP_RIGHT_WHEEL_HW_STATE = 0.0;
  double EXP_LEFT_WHEEL_REF = 0.0;
  double EXP_RIGHT_WHEEL_REF = 0.0;
  double EXP_STATE_ODOM_X = 0.0;
  double EXP_STATE_ODOM_Y = 0.0;

  // Expected behaviors struct used in chaining activation/deactivation tests
  struct ExpectedBehaviorStruct
  {
    controller_interface::return_type return_type;
    std::future_status future_status;
    uint8_t state;
  };
};

// The tests are implementing example of chained-control for DiffDrive robot shown here:
// https://github.com/ros-controls/roadmap/blob/9f32e215a84347aee0b519cb24d081f23bbbb224/design_drafts/cascade_control.md#motivation-purpose-and-use
// The controller have the names as stated in figure, but they are simply forwarding values without
// functionality that their name would suggest
TEST_P(TestControllerChainingWithControllerManager, test_chained_controllers)
{
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  SetToChainedModeAndMakeInterfacesAvailable(
    pid_left_wheel_controller, PID_LEFT_WHEEL, {}, PID_LEFT_WHEEL_REFERENCE_INTERFACES);
  SetToChainedModeAndMakeInterfacesAvailable(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, {}, PID_RIGHT_WHEEL_REFERENCE_INTERFACES);
  SetToChainedModeAndMakeInterfacesAvailable(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_STATE_INTERFACES,
    DIFF_DRIVE_REFERENCE_INTERFACES);
  SetToChainedModeAndMakeInterfacesAvailable(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, SENSOR_FUSION_ESIMTATED_INTERFACES, {});

  EXPECT_THROW(
    cm_->resource_manager_->make_controller_reference_interfaces_available(
      POSITION_TRACKING_CONTROLLER),
    std::out_of_range);

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);

  // Before starting check that the internal counters are set to zero
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 0u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 0u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 0u);
  ASSERT_EQ(diff_drive_controller_two->internal_counter, 0u);
  ASSERT_EQ(position_tracking_controller->internal_counter, 0u);
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 0u);
  ASSERT_EQ(odom_publisher_controller->internal_counter, 0u);
  ASSERT_EQ(robot_localization_controller->internal_counter, 0u);

  // activate controllers - CONTROLLERS HAVE TO ADDED REVERSE EXECUTION ORDER
  // (otherwise, interface will be missing)
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 1u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 3u);

  // Diff-Drive Controller claims the reference interfaces of PID controllers
  ActivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 1u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 3u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 5u);

  // Position-Tracking Controller uses reference interfaces of Diff-Drive Controller
  ActivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(position_tracking_controller->internal_counter, 1u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 3u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 5u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 7u);

  // Sensor Controller uses exported state interfaces of Diff-Drive Controller and IMU
  ActivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 1u);
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 1u);
  ASSERT_EQ(position_tracking_controller->internal_counter, 3u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 5u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 7u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 9u);

  // Robot localization Controller uses exported state interfaces of Diff-Drive Controller
  ActivateAndCheckController(robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 1u);
  ASSERT_EQ(robot_localization_controller->internal_counter, 1u);
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 3u);
  ASSERT_EQ(position_tracking_controller->internal_counter, 5u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 7u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 9u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 11u);

  // Odometry Publisher Controller uses exported state interfaces of Diff-Drive Controller
  ActivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 1u);
  // 'rot_z' reference interface is not claimed
  for (const auto & interface : {"diff_drive_controller/rot_z"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }
  ASSERT_EQ(odom_publisher_controller->internal_counter, 1u);
  ASSERT_EQ(robot_localization_controller->internal_counter, 3u);
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 5u);
  ASSERT_EQ(position_tracking_controller->internal_counter, 7u);
  ASSERT_EQ(diff_drive_controller->internal_counter, 9u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 11u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 13u);

  // update controllers
  std::vector<double> reference = {32.0, 128.0};

  // update 'Position Tracking' controller
  for (auto & value : diff_drive_controller->reference_interfaces_)
  {
    ASSERT_EQ(value, 0.0);  // default reference values are 0.0
  }
  position_tracking_controller->external_commands_for_testing_[0] = reference[0];
  position_tracking_controller->external_commands_for_testing_[1] = reference[1];
  position_tracking_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(position_tracking_controller->internal_counter, 8u);

  ASSERT_EQ(diff_drive_controller->reference_interfaces_[0], reference[0]);  // position_controller
  ASSERT_EQ(diff_drive_controller->reference_interfaces_[1], reference[1]);  // is pass-through

  // update 'Diff Drive' Controller
  diff_drive_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(diff_drive_controller->internal_counter, 10u);
  // default reference values are 0.0 - they should be changed now
  EXP_LEFT_WHEEL_REF = chained_ctrl_calculation(reference[0], EXP_LEFT_WHEEL_HW_STATE);    // 32-0
  EXP_RIGHT_WHEEL_REF = chained_ctrl_calculation(reference[1], EXP_RIGHT_WHEEL_HW_STATE);  // 128-0
  ASSERT_EQ(pid_left_wheel_controller->reference_interfaces_[0], EXP_LEFT_WHEEL_REF);
  ASSERT_EQ(pid_right_wheel_controller->reference_interfaces_[0], EXP_RIGHT_WHEEL_REF);

  // run the update cycles of the robot localization and odom publisher controller
  sensor_fusion_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 6u);
  robot_localization_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(robot_localization_controller->internal_counter, 4u);
  odom_publisher_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(odom_publisher_controller->internal_counter, 2u);
  EXP_STATE_ODOM_X =
    chained_estimate_calculation(reference[0], EXP_LEFT_WHEEL_HW_STATE);  // 32-0 / dt
  EXP_STATE_ODOM_Y =
    chained_estimate_calculation(reference[1], EXP_RIGHT_WHEEL_HW_STATE);  // 128-0 / dt
  ASSERT_EQ(robot_localization_controller->get_state_interface_data().size(), 3u);
  ASSERT_EQ(robot_localization_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
  ASSERT_EQ(robot_localization_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);
  ASSERT_EQ(odom_publisher_controller->get_state_interface_data().size(), 2u);
  ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
  ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);

  // update PID controllers that are writing to hardware
  pid_left_wheel_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 14u);
  pid_right_wheel_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 12u);

  // update hardware ('read' is  sufficient for test hardware)
  cm_->resource_manager_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  // 32 - 0
  EXP_LEFT_WHEEL_CMD = chained_ctrl_calculation(EXP_LEFT_WHEEL_REF, EXP_LEFT_WHEEL_HW_STATE);
  // 32 / 2
  EXP_LEFT_WHEEL_HW_STATE = hardware_calculation(EXP_LEFT_WHEEL_CMD);
  ASSERT_EQ(pid_left_wheel_controller->command_interfaces_[0].get_value(), EXP_LEFT_WHEEL_CMD);
  ASSERT_EQ(pid_left_wheel_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);
  // DiffDrive uses the same state
  ASSERT_EQ(diff_drive_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);
  // The state doesn't change wrt to any data from the hardware calculation
  ASSERT_EQ(robot_localization_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);
  ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[0], EXP_STATE_ODOM_X);

  // 128 - 0
  EXP_RIGHT_WHEEL_CMD = chained_ctrl_calculation(EXP_RIGHT_WHEEL_REF, EXP_RIGHT_WHEEL_HW_STATE);
  // 128 / 2
  EXP_RIGHT_WHEEL_HW_STATE = hardware_calculation(EXP_RIGHT_WHEEL_CMD);
  ASSERT_EQ(pid_right_wheel_controller->command_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_CMD);
  ASSERT_EQ(pid_right_wheel_controller->state_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_HW_STATE);
  ASSERT_EQ(odom_publisher_controller->internal_counter, 2u);
  ASSERT_EQ(sensor_fusion_controller->internal_counter, 6u);
  ASSERT_EQ(robot_localization_controller->internal_counter, 4u);
  // DiffDrive uses the same state
  ASSERT_EQ(diff_drive_controller->state_interfaces_[1].get_value(), EXP_RIGHT_WHEEL_HW_STATE);
  // The state doesn't change wrt to any data from the hardware calculation
  ASSERT_EQ(robot_localization_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);
  ASSERT_EQ(odom_publisher_controller->get_state_interface_data()[1], EXP_STATE_ODOM_Y);

  // update all controllers at once and see that all have expected values --> also checks the order
  // of controller execution

  reference = {1024.0, 4096.0};
  UpdateAllControllerAndCheck(reference, 3u);

  // TODO(destogl): Add here also slow disabling of controllers

  // TODO(destogl): Activate test parameter use
}

TEST_P(
  TestControllerChainingWithControllerManager, test_chained_controllers_auto_switch_to_chained_mode)
{
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  rclcpp::get_logger("ControllerManager::utils").set_level(rclcpp::Logger::Level::Debug);

  // at beginning controllers are not in chained mode
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());

  // still not in chained mode because no preceding controller is activated
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // DiffDrive (preceding) controller is activated --> PID controller in chained mode
  ActivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 1u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // PositionController is activated --> all following controller in chained mode
  ActivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 1u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_TRUE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  for (const auto & interface : {"diff_drive_controller/vel_x", "diff_drive_controller/vel_y"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }
  for (const auto & interface : {"diff_drive_controller/odom_x", "diff_drive_controller/odom_y"})
  {
    EXPECT_TRUE(cm_->resource_manager_->state_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->state_interface_is_available(interface));
  }

  // Sensor fusion Controller uses exported state interfaces of Diff-Drive Controller and IMU
  ActivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 1u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_TRUE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // Robot localization Controller uses exported state interfaces of sensor fusion Controller
  ActivateAndCheckController(robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 1u);

  // Odometry Publisher Controller uses exported state interfaces of Diff-Drive Controller
  ActivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 1u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_TRUE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  ASSERT_EQ(robot_localization_controller->internal_counter, 3u);
  ASSERT_EQ(odom_publisher_controller->internal_counter, 1u);

  UpdateAllControllerAndCheck({32.0, 128.0}, 2u);
  UpdateAllControllerAndCheck({1024.0, 4096.0}, 3u);

  // Test switch 'from chained mode' when controllers are deactivated

  // PositionController is deactivated --> DiffDrive controller still continues in chained mode
  // As the DiffDriveController is in chained mode, right now we tend to also deactivate
  // the other controllers that rely on the DiffDriveController exported interfaces
  DeactivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 10u, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  // SensorFusionController continues to stay in the chained mode as it is still using the state
  // interfaces
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the robot localization controller and see that the sensor fusion controller is still
  // active but not in the chained mode
  DeactivateAndCheckController(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 8u, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the odometry publisher controller
  DeactivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 8u, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the sensor_fusion controller and see that the diff_drive_controller is still active
  // but not in the chained mode
  DeactivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 14u, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the diff_drive_controller as all it's following controllers that uses it's
  // interfaces are deactivated
  DeactivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 20u, true);
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());

  // all controllers are deactivated --> chained mode is not changed
  DeactivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 26u, true);
  DeactivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 26u, true);
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
}

TEST_P(
  TestControllerChainingWithControllerManager, test_chained_controllers_activation_error_handling)
{
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  rclcpp::get_logger("ControllerManager::utils").set_level(rclcpp::Logger::Level::Debug);

  // at beginning controllers are not in chained mode
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // Test Case 1: Trying to activate a preceding controller when following controller
  // is not activated --> return error (If STRICT); Preceding controller is still inactive.

  static std::unordered_map<int32_t, ExpectedBehaviorStruct> expected = {
    {controller_manager_msgs::srv::SwitchController::Request::STRICT,
     {controller_interface::return_type::ERROR, std::future_status::ready,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE}},
    {controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
     {controller_interface::return_type::OK, std::future_status::timeout,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE}}};

  // Attempt to activate preceding controller (diff-drive controller) with no check
  ActivateController(
    DIFF_DRIVE_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);

  // Check if the controller activated (Should not be activated)
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());

  ActivateController(
    SENSOR_FUSION_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);
  // Check if the controller activated (Should not be activated)
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());

  // Test Case 2: Try to activate a preceding controller the same time when trying to
  // deactivate a following controller (using switch_controller function)
  // --> return error; preceding controller is not activated,
  // BUT following controller IS deactivated

  // Activate and check the following controllers:
  // (pid_left_wheel_controller) (pid_right_wheel_controller)
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);

  // Attempt to activate a preceding controller (diff-drive controller + remaining)
  // while trying to deactivate a following controller
  switch_test_controllers(
    {DIFF_DRIVE_CONTROLLER, ODOM_PUBLISHER_CONTROLLER, SENSOR_FUSION_CONTROLLER}, {PID_RIGHT_WHEEL},
    test_param.strictness, expected.at(test_param.strictness).future_status,
    expected.at(test_param.strictness).return_type);

  // Preceding controller should stay deactivated and following controller
  // should be deactivated (if BEST_EFFORT)
  // If STRICT, preceding controller should stay deactivated and following controller
  // should stay activated
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
}

TEST_P(
  TestControllerChainingWithControllerManager,
  test_chained_controllers_activation_switching_error_handling)
{
  // Test Case 3: In terms of current implementation.
  // Example: Need two diff drive controllers, one should be deactivated,
  // and the other should be activated. Following controller should stay in activated state.
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  rclcpp::get_logger("ControllerManager::utils").set_level(rclcpp::Logger::Level::Debug);

  // Activate the following controller and the preceding controllers
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 1u);
  ActivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 1u);

  // Verify that the other preceding controller is deactivated (diff_drive_controller_two) and other
  // depending controllers are active
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller_two->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    position_tracking_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());

  // Deactivate position_tracking_controller and activate position_tracking_controller_two
  switch_test_controllers(
    {POSITION_TRACKING_CONTROLLER_TWO}, {POSITION_TRACKING_CONTROLLER}, test_param.strictness,
    std::future_status::timeout, controller_interface::return_type::OK);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());

  // Now deactivate the position_tracking_controller_two and it should be in inactive state
  switch_test_controllers(
    {}, {POSITION_TRACKING_CONTROLLER_TWO}, test_param.strictness, std::future_status::timeout,
    controller_interface::return_type::OK);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());

  // Activate it again and deactivate it others to see if we can deactivate it in a group
  switch_test_controllers(
    {POSITION_TRACKING_CONTROLLER_TWO}, {}, test_param.strictness, std::future_status::timeout,
    controller_interface::return_type::OK);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());

  // Deactivate the first preceding controller (diff_drive_controller) and
  // activate the other preceding controller (diff_drive_controller_two)
  switch_test_controllers(
    {DIFF_DRIVE_CONTROLLER_TWO},
    {POSITION_TRACKING_CONTROLLER_TWO, DIFF_DRIVE_CONTROLLER, SENSOR_FUSION_CONTROLLER,
     ROBOT_LOCALIZATION_CONTROLLER, ODOM_PUBLISHER_CONTROLLER},
    test_param.strictness, std::future_status::timeout, controller_interface::return_type::OK);

  // Following controllers should stay active
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  // The original preceding controller (diff_drive_controller) should be inactive while
  // the other preceding controller should be active (diff_drive_controller_two)
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller_two->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());

  // Activate all the controllers again in group and deactivate the diff_drive_controller_two
  switch_test_controllers(
    {POSITION_TRACKING_CONTROLLER_TWO, DIFF_DRIVE_CONTROLLER, SENSOR_FUSION_CONTROLLER,
     ROBOT_LOCALIZATION_CONTROLLER, ODOM_PUBLISHER_CONTROLLER},
    {DIFF_DRIVE_CONTROLLER_TWO}, test_param.strictness, std::future_status::timeout,
    controller_interface::return_type::OK);
  // Following controllers should stay active
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_TRUE(diff_drive_controller->is_in_chained_mode());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller_two->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  // This is false, because it only uses the state interfaces and exposes state interfaces
  EXPECT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    position_tracking_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    position_tracking_controller_two->get_lifecycle_state().id());
}

TEST_P(
  TestControllerChainingWithControllerManager, test_chained_controllers_deactivation_error_handling)
{
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  rclcpp::get_logger("ControllerManager::utils").set_level(rclcpp::Logger::Level::Debug);

  // at beginning controllers are not in chained mode
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // Activate following controllers
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);

  // Test Case 5: Deactivating a preceding controller that is not active --> return error;
  // all controller stay in the same state

  // There is different error and timeout behavior depending on strictness
  static std::unordered_map<int32_t, ExpectedBehaviorStruct> expected = {
    {controller_manager_msgs::srv::SwitchController::Request::STRICT,
     {controller_interface::return_type::ERROR, std::future_status::ready,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE}},
    {controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
     {controller_interface::return_type::OK, std::future_status::timeout,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE}}};

  // Verify preceding controller (diff_drive_controller) is inactive
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Attempt to deactivate inactive controller (diff_drive_controller)
  DeactivateController(
    DIFF_DRIVE_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);
  DeactivateController(
    SENSOR_FUSION_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);
  DeactivateController(
    ODOM_PUBLISHER_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);
  DeactivateController(
    ROBOT_LOCALIZATION_CONTROLLER, expected.at(test_param.strictness).return_type,
    std::future_status::ready);

  // Check to see preceding controller (diff_drive_controller) is still inactive and
  // following controllers (pid_left_wheel_controller) (pid_left_wheel_controller) are still active
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Test Case 6: following controller is deactivated but preceding controller will be activated
  // --> return error; controllers stay in the same state

  switch_test_controllers(
    {DIFF_DRIVE_CONTROLLER}, {PID_LEFT_WHEEL, PID_RIGHT_WHEEL}, test_param.strictness,
    expected.at(test_param.strictness).future_status,
    expected.at(test_param.strictness).return_type);

  // Preceding controller should stay deactivated and following controller
  // should be deactivated (if BEST_EFFORT)
  // If STRICT, preceding controller should stay deactivated and following controller
  // should stay activated
  EXPECT_EQ(
    expected.at(test_param.strictness).state,
    pid_right_wheel_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    expected.at(test_param.strictness).state,
    pid_left_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());

  // Test Case 7: following controller deactivation but preceding controller is active
  // --> return error; controllers stay in the same state as they were

  // Activate all controllers for this test
  ActivateController(
    PID_LEFT_WHEEL, expected.at(test_param.strictness).return_type,
    expected.at(test_param.strictness).future_status);
  ActivateController(
    PID_RIGHT_WHEEL, expected.at(test_param.strictness).return_type,
    expected.at(test_param.strictness).future_status);
  ActivateController(
    DIFF_DRIVE_CONTROLLER, controller_interface::return_type::OK, std::future_status::timeout);

  // Expect all controllers to be active
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());

  // Attempt to deactivate following controllers
  switch_test_controllers(
    {}, {PID_LEFT_WHEEL, PID_RIGHT_WHEEL}, test_param.strictness, std::future_status::ready,
    expected.at(test_param.strictness).return_type);

  // All controllers should still be active
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());

  // Attempt to deactivate a following controller
  switch_test_controllers(
    {}, {PID_RIGHT_WHEEL}, test_param.strictness, std::future_status::ready,
    expected.at(test_param.strictness).return_type);

  // All controllers should still be active
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
}

TEST_P(
  TestControllerChainingWithControllerManager,
  test_chained_controllers_deactivation_switching_error_handling)
{
  SetupControllers();

  // add all controllers - CONTROLLERS HAVE TO ADDED IN EXECUTION ORDER
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();
  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  rclcpp::get_logger("ControllerManager::utils").set_level(rclcpp::Logger::Level::Debug);

  // at beginning controllers are not in chained mode
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // Activate the following controller and the preceding controllers
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 1u);
  ActivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 1u);

  // check once active that they are in chained mode
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());

  // Verify that initially all of them are in active state
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_left_wheel_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    pid_right_wheel_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    position_tracking_controller->get_lifecycle_state().id());

  // There is different error and timeout behavior depending on strictness
  static std::unordered_map<int32_t, ExpectedBehaviorStruct> expected = {
    {controller_manager_msgs::srv::SwitchController::Request::STRICT,
     {controller_interface::return_type::ERROR, std::future_status::ready,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE}},
    {controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
     {controller_interface::return_type::OK, std::future_status::ready,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE}}};

  // Test switch 'from chained mode' when controllers are deactivated and possible combination of
  // disabling controllers that use reference/state interfaces of the other controller. This is
  // important to check that deactivation is not trigger irrespective of the type
  // (reference/state) interface that is shared among the other controllers

  // PositionController is deactivated --> DiffDrive controller still continues in chained mode
  // As the DiffDriveController is in chained mode, right now we tend to also deactivate
  // the other controllers that rely on the DiffDriveController expected interfaces
  DeactivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 2u, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(diff_drive_controller->is_in_chained_mode());
  EXPECT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  // SensorFusionController continues to stay in the chained mode as it is still using the state
  // interfaces
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // DiffDrive (preceding) controller is activated --> PID controller in chained mod
  // Let's try to deactivate the diff_drive_control, it should fail as there are still other
  // controllers that use it's resources
  DeactivateController(
    DIFF_DRIVE_CONTROLLER, expected.at(test_param.strictness).return_type,
    expected.at(test_param.strictness).future_status);
  check_after_de_activate(
    diff_drive_controller, DIFF_DRIVE_CLAIMED_INTERFACES, 0u,
    controller_interface::return_type::ERROR, true, true);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  EXPECT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Trying to deactivate the sensor fusion controller, however, it won't be deactivated as the
  // robot localization controller is still active
  DeactivateAndCheckController(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 0u, true,
    expected.at(test_param.strictness).return_type,
    expected.at(test_param.strictness).future_status);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  EXPECT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the robot localization controller and see that the sensor fusion controller is still
  // active but not in the chained mode
  DeactivateAndCheckController(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 0u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the sensor_fusion controller and this should be successful as there are no other
  // controllers using it's interfaces
  DeactivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 0u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  ASSERT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the odometry publisher controller and now the diff_drive should continue active but
  // not in chained mode
  DeactivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 0u);
  EXPECT_TRUE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_TRUE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());

  // Deactivate the diff_drive_controller as all it's following controllers that uses it's
  // interfaces are deactivated
  DeactivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 0u, true);
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());

  // all controllers are deactivated --> chained mode is not changed
  DeactivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 0u, true);
  DeactivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 0u, true);
  EXPECT_FALSE(pid_left_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(pid_right_wheel_controller->is_in_chained_mode());
  EXPECT_FALSE(diff_drive_controller->is_in_chained_mode());
  ASSERT_FALSE(sensor_fusion_controller->is_in_chained_mode());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    diff_drive_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    sensor_fusion_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    odom_publisher_controller->get_lifecycle_state().id());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    robot_localization_controller->get_lifecycle_state().id());
}

TEST_P(TestControllerChainingWithControllerManager, test_chained_controllers_adding_in_random_order)
{
  SetupControllers();

  // add all controllers in random order to test the sorting
  cm_->add_controller(
    pid_left_wheel_controller, PID_LEFT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    pid_right_wheel_controller, PID_RIGHT_WHEEL,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    diff_drive_controller_two, DIFF_DRIVE_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sensor_fusion_controller, SENSOR_FUSION_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    position_tracking_controller_two, POSITION_TRACKING_CONTROLLER_TWO,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);

  CheckIfControllersAreAddedCorrectly();

  ConfigureAndCheckControllers();

  SetToChainedModeAndMakeInterfacesAvailable(
    pid_left_wheel_controller, PID_LEFT_WHEEL, {}, PID_LEFT_WHEEL_REFERENCE_INTERFACES);
  SetToChainedModeAndMakeInterfacesAvailable(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, {}, PID_RIGHT_WHEEL_REFERENCE_INTERFACES);
  SetToChainedModeAndMakeInterfacesAvailable(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_STATE_INTERFACES,
    DIFF_DRIVE_REFERENCE_INTERFACES);

  EXPECT_THROW(
    cm_->resource_manager_->make_controller_reference_interfaces_available(
      POSITION_TRACKING_CONTROLLER),
    std::out_of_range);

  // Set ControllerManager into Debug-Mode output to have detailed output on updating controllers
  cm_->get_logger().set_level(rclcpp::Logger::Level::Debug);

  // activate controllers - CONTROLLERS HAVE TO ADDED REVERSE EXECUTION ORDER
  // (otherwise, interface will be missing)
  ActivateAndCheckController(
    pid_left_wheel_controller, PID_LEFT_WHEEL, PID_LEFT_WHEEL_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(
    pid_right_wheel_controller, PID_RIGHT_WHEEL, PID_RIGHT_WHEEL_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 3u);

  // Diff-Drive Controller claims the reference interfaces of PID controllers
  ActivateAndCheckController(
    diff_drive_controller, DIFF_DRIVE_CONTROLLER, DIFF_DRIVE_CLAIMED_INTERFACES, 1u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 3u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 5u);

  // Position-Tracking Controller uses reference interfaces of Diff-Drive Controller
  ActivateAndCheckController(
    position_tracking_controller, POSITION_TRACKING_CONTROLLER,
    POSITION_CONTROLLER_CLAIMED_INTERFACES, 1u);
  ActivateAndCheckController(sensor_fusion_controller, SENSOR_FUSION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(robot_localization_controller, ROBOT_LOCALIZATION_CONTROLLER, {}, 1u);
  ActivateAndCheckController(odom_publisher_controller, ODOM_PUBLISHER_CONTROLLER, {}, 1u);
  // 'rot_z' reference interface is not claimed
  for (const auto & interface : {"diff_drive_controller/rot_z"})
  {
    EXPECT_TRUE(cm_->resource_manager_->command_interface_exists(interface));
    EXPECT_TRUE(cm_->resource_manager_->command_interface_is_available(interface));
    EXPECT_FALSE(cm_->resource_manager_->command_interface_is_claimed(interface));
  }
  ASSERT_EQ(diff_drive_controller->internal_counter, 9u);
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 11u);
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 13u);

  // update controllers
  std::vector<double> reference = {32.0, 128.0};

  sensor_fusion_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  robot_localization_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  odom_publisher_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // update 'Position Tracking' controller
  for (auto & value : diff_drive_controller->reference_interfaces_)
  {
    ASSERT_EQ(value, 0.0);  // default reference values are 0.0
  }
  position_tracking_controller->external_commands_for_testing_[0] = reference[0];
  position_tracking_controller->external_commands_for_testing_[1] = reference[1];
  position_tracking_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(position_tracking_controller->internal_counter, 8u);

  ASSERT_EQ(diff_drive_controller->reference_interfaces_[0], reference[0]);  // position_controller
  ASSERT_EQ(diff_drive_controller->reference_interfaces_[1], reference[1]);  // is pass-through

  // update 'Diff Drive' Controller
  diff_drive_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(diff_drive_controller->internal_counter, 10u);
  // default reference values are 0.0 - they should be changed now
  EXP_LEFT_WHEEL_REF = chained_ctrl_calculation(reference[0], EXP_LEFT_WHEEL_HW_STATE);    // 32-0
  EXP_RIGHT_WHEEL_REF = chained_ctrl_calculation(reference[1], EXP_RIGHT_WHEEL_HW_STATE);  // 128-0
  ASSERT_EQ(pid_left_wheel_controller->reference_interfaces_[0], EXP_LEFT_WHEEL_REF);
  ASSERT_EQ(pid_right_wheel_controller->reference_interfaces_[0], EXP_RIGHT_WHEEL_REF);

  // update PID controllers that are writing to hardware
  pid_left_wheel_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(pid_left_wheel_controller->internal_counter, 14u);
  pid_right_wheel_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  ASSERT_EQ(pid_right_wheel_controller->internal_counter, 12u);

  // update hardware ('read' is  sufficient for test hardware)
  cm_->resource_manager_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  // 32 - 0
  EXP_LEFT_WHEEL_CMD = chained_ctrl_calculation(EXP_LEFT_WHEEL_REF, EXP_LEFT_WHEEL_HW_STATE);
  // 32 / 2
  EXP_LEFT_WHEEL_HW_STATE = hardware_calculation(EXP_LEFT_WHEEL_CMD);
  ASSERT_EQ(pid_left_wheel_controller->command_interfaces_[0].get_value(), EXP_LEFT_WHEEL_CMD);
  ASSERT_EQ(pid_left_wheel_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);
  // DiffDrive uses the same state
  ASSERT_EQ(diff_drive_controller->state_interfaces_[0].get_value(), EXP_LEFT_WHEEL_HW_STATE);

  // 128 - 0
  EXP_RIGHT_WHEEL_CMD = chained_ctrl_calculation(EXP_RIGHT_WHEEL_REF, EXP_RIGHT_WHEEL_HW_STATE);
  // 128 / 2
  EXP_RIGHT_WHEEL_HW_STATE = hardware_calculation(EXP_RIGHT_WHEEL_CMD);
  ASSERT_EQ(pid_right_wheel_controller->command_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_CMD);
  ASSERT_EQ(pid_right_wheel_controller->state_interfaces_[0].get_value(), EXP_RIGHT_WHEEL_HW_STATE);
  // DiffDrive uses the same state
  ASSERT_EQ(diff_drive_controller->state_interfaces_[1].get_value(), EXP_RIGHT_WHEEL_HW_STATE);

  // update all controllers at once and see that all have expected values --> also checks the order
  // of controller execution

  reference = {1024.0, 4096.0};
  UpdateAllControllerAndCheck(reference, 3u);
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestControllerChainingWithControllerManager,
  testing::Values(strict, best_effort));
