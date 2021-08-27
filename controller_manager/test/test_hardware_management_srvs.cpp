// Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/msg/hardware_components_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/manage_hardware_activity.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/parameter.hpp"

using ::testing::_;
using ::testing::Return;

using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_ACTUATOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SENSOR_HARDWARE_TYPE;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_CLASS_TYPE;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_NAME;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_STATE_INTERFACES;
using ros2_control_test_assets::TEST_SYSTEM_HARDWARE_TYPE;

using LFC_STATE = lifecycle_msgs::msg::State;

using namespace std::chrono_literals;

// TODO(destogl): Use somewhere pre-configured strings later
const auto HW_STATE_UNCONFIGURED = "unconfigured";
const auto HW_STATE_ACTIVE = "active";
const auto HW_STATE_INACTIVE = "inactive";
const auto HW_STATE_FINALIZED = "finalized";

class TestControllerManagerHWManagementSrvs : public TestControllerManagerSrvs
{
public:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(), executor_, TEST_CM_NAME);
    run_updater_ = false;

    cm_->set_parameter(
      rclcpp::Parameter("robot_description", ros2_control_test_assets::minimal_robot_urdf));
    cm_->set_parameter(rclcpp::Parameter(
      "autostart_components", std::vector<std::string>({TEST_ACTUATOR_HARDWARE_NAME})));

    cm_->init_resource_manager();

    SetUpSrvsCMExecutor();
  }

  void check_component_fileds(
    const controller_manager_msgs::msg::HardwareComponentsState & component,
    const std::string & name, const std::string & type, const std::string & class_type,
    const uint8_t state_id, const std::string & state_label)
  {
    EXPECT_EQ(component.name, name);
    EXPECT_EQ(component.type, type);
    EXPECT_EQ(component.class_type, class_type);
    EXPECT_EQ(component.state.id, state_id);
    EXPECT_EQ(component.state.label, state_label);
  }

  void list_hardware_components_and_check(
    const std::vector<uint8_t> & hw_state_ids, const std::vector<std::string> & hw_state_labels,
    const std::vector<std::vector<std::vector<bool>>> & hw_itfs_claimed_status)
  {
    rclcpp::executors::SingleThreadedExecutor srv_executor;
    rclcpp::Node::SharedPtr list_srv_node = std::make_shared<rclcpp::Node>("list_srv_client");
    srv_executor.add_node(list_srv_node);
    rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_client =
      list_srv_node->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
        std::string(TEST_CM_NAME) + "/list_hardware_components");
    auto request =
      std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>();

    auto result = call_service_and_wait(*list_client, request, srv_executor);

    auto check_interfaces =
      [](
        const std::vector<controller_manager_msgs::msg::HardwareInterface> & interfaces,
        const std::vector<const char *> & interface_names,
        const std::vector<bool> is_claimed_status) {
        for (auto i = 0ul; i < interfaces.size(); ++i)
        {
          auto it = std::find(interface_names.begin(), interface_names.end(), interfaces[i].name);
          EXPECT_NE(it, interface_names.end());
          EXPECT_EQ(interfaces[i].is_claimed, is_claimed_status[i]);
        }
      };

    for (const auto & component : result->component)
    {
      if (component.name == TEST_ACTUATOR_HARDWARE_NAME)
      {
        check_component_fileds(
          component, TEST_ACTUATOR_HARDWARE_NAME, TEST_ACTUATOR_HARDWARE_TYPE,
          TEST_ACTUATOR_HARDWARE_CLASS_TYPE, hw_state_ids[0], hw_state_labels[0]);
        check_interfaces(
          component.command_interfaces, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
          hw_itfs_claimed_status[0][0]);
        check_interfaces(
          component.state_interfaces, TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
          hw_itfs_claimed_status[0][1]);
      }
      if (component.name == TEST_SENSOR_HARDWARE_NAME)
      {
        check_component_fileds(
          component, TEST_SENSOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_TYPE,
          TEST_SENSOR_HARDWARE_CLASS_TYPE, hw_state_ids[1], hw_state_labels[1]);
        check_interfaces(
          component.command_interfaces, TEST_SENSOR_HARDWARE_COMMAND_INTERFACES,
          hw_itfs_claimed_status[1][0]);
        check_interfaces(
          component.state_interfaces, TEST_SENSOR_HARDWARE_STATE_INTERFACES,
          hw_itfs_claimed_status[1][1]);
      }
      if (component.name == TEST_SYSTEM_HARDWARE_NAME)
      {
        check_component_fileds(
          component, TEST_SYSTEM_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_TYPE,
          TEST_SYSTEM_HARDWARE_CLASS_TYPE, hw_state_ids[2], hw_state_labels[2]);
        check_interfaces(
          component.command_interfaces, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
          hw_itfs_claimed_status[2][0]);
        check_interfaces(
          component.state_interfaces, TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
          hw_itfs_claimed_status[2][1]);
      }
    }
  }

  bool manage_hardware_activity(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    rclcpp::executors::SingleThreadedExecutor srv_executor;
    rclcpp::Node::SharedPtr mha_srv_node = std::make_shared<rclcpp::Node>("mha_srv_client");
    srv_executor.add_node(mha_srv_node);
    rclcpp::Client<controller_manager_msgs::srv::ManageHardwareActivity>::SharedPtr mha_client =
      mha_srv_node->create_client<controller_manager_msgs::srv::ManageHardwareActivity>(
        std::string(TEST_CM_NAME) + "/manage_hardware_activity");
    auto request =
      std::make_shared<controller_manager_msgs::srv::ManageHardwareActivity::Request>();

    request->activate = activate;
    request->deactivate = deactivate;

    auto result = call_service_and_wait(*mha_client, request, srv_executor);

    return result->ok;
  }
};

TEST_F(TestControllerManagerHWManagementSrvs, list_hardware_components)
{
  // Default status after start - checks also if "autostart_parameter is correctly read"

  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_INACTIVE}),
    std::vector<std::string>({HW_STATE_ACTIVE, HW_STATE_INACTIVE, HW_STATE_INACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));
}

TEST_F(TestControllerManagerHWManagementSrvs, selective_start_components)
{
  // Default status after start
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_INACTIVE}),
    std::vector<std::string>({HW_STATE_ACTIVE, HW_STATE_INACTIVE, HW_STATE_INACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));

  // Activate system
  manage_hardware_activity(
    {TEST_SYSTEM_HARDWARE_NAME},  // activate
    {}                            // deactivate
  );
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({HW_STATE_ACTIVE, HW_STATE_INACTIVE, HW_STATE_ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));

  // Deactivate actuator; Activate sensor
  manage_hardware_activity(
    {TEST_SENSOR_HARDWARE_NAME},   // activate
    {TEST_ACTUATOR_HARDWARE_NAME}  // deactivate
  );
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({HW_STATE_INACTIVE, HW_STATE_ACTIVE, HW_STATE_ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));

  // Double activate system
  // TODO(destogl): add strictness test
  manage_hardware_activity(
    {TEST_SYSTEM_HARDWARE_NAME},  // activate
    {}                            // deactivate
  );
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({HW_STATE_INACTIVE, HW_STATE_ACTIVE, HW_STATE_ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));

  // Double deactivate actuator
  // TODO(destogl): add strictness test
  manage_hardware_activity(
    {},                            // activate
    {TEST_ACTUATOR_HARDWARE_NAME}  // deactivate
  );
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({HW_STATE_INACTIVE, HW_STATE_ACTIVE, HW_STATE_ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      {{false}, {false, false}},         // actuator
      {{}, {false}},                     // sensor
      {{false, false}, {false, false}},  // system
    }));

  //   auto test_controller = std::make_shared<test_controller::TestController>();
  //   cm_->add_controller(
  //     test_controller, test_controller::TEST_CONTROLLER_NAME,
  //     test_controller::TEST_CONTROLLER_CLASS_NAME);
  //   EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  //   EXPECT_EQ(2, test_controller.use_count());
  //
  //   EXPECT_EQ(controller_interface::return_type::OK, cm_->update());
  //   EXPECT_EQ(
  //     0u,
  //     test_controller->internal_counter) <<
  //     "Update should not reach an unconfigured controller";
  //
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
  //     test_controller->get_current_state().id());
  //
  //   // configure controller
  //   cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  //   EXPECT_EQ(controller_interface::return_type::OK, cm_->update());
  //   EXPECT_EQ(0u, test_controller->internal_counter) << "Controller is not started";
  //
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
  //     test_controller->get_current_state().id());
  //
  //   // Start controller, will take effect at the end of the update function
  //   std::vector<std::string> start_controllers = {test_controller::TEST_CONTROLLER_NAME};
  //   std::vector<std::string> stop_controllers = {};
  //   auto switch_future = std::async(
  //     std::launch::async,
  //     &controller_manager::ControllerManager::switch_controller, cm_,
  //     start_controllers, stop_controllers,
  //     STRICT, true, rclcpp::Duration(0, 0));
  //
  //   ASSERT_EQ(
  //     std::future_status::timeout,
  //     switch_future.wait_for(std::chrono::milliseconds(100))) <<
  //     "switch_controller should be blocking until next update cycle";
  //
  //   EXPECT_EQ(controller_interface::return_type::OK, cm_->update());
  //   EXPECT_EQ(0u, test_controller->internal_counter) <<
  //   "Controller is started at the end of update";
  //   {
  //     ControllerManagerRunner cm_runner(this);
  //     EXPECT_EQ(
  //       controller_interface::return_type::OK,
  //       switch_future.get()
  //     );
  //   }
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
  //     test_controller->get_current_state().id());
  //
  //   EXPECT_EQ(controller_interface::return_type::OK, cm_->update());
  //   EXPECT_GE(test_controller->internal_counter, 1u);
  //   auto last_internal_counter = test_controller->internal_counter;
  //
  //   // Stop controller, will take effect at the end of the update function
  //   start_controllers = {};
  //   stop_controllers = {test_controller::TEST_CONTROLLER_NAME};
  //   switch_future = std::async(
  //     std::launch::async,
  //     &controller_manager::ControllerManager::switch_controller, cm_,
  //     start_controllers, stop_controllers,
  //     STRICT, true, rclcpp::Duration(0, 0));
  //
  //   ASSERT_EQ(
  //     std::future_status::timeout,
  //     switch_future.wait_for(std::chrono::milliseconds(100))) <<
  //     "switch_controller should be blocking until next update cycle";
  //
  //   EXPECT_EQ(controller_interface::return_type::OK, cm_->update());
  //   EXPECT_EQ(
  //     last_internal_counter + 1u,
  //     test_controller->internal_counter) <<
  //     "Controller is stopped at the end of update, so it should have done one more update";
  //   {
  //     ControllerManagerRunner cm_runner(this);
  //     EXPECT_EQ(
  //       controller_interface::return_type::OK,
  //       switch_future.get()
  //     );
  //   }
  //
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
  //     test_controller->get_current_state().id());
  //   auto unload_future = std::async(
  //     std::launch::async,
  //     &controller_manager::ControllerManager::unload_controller, cm_,
  //     test_controller::TEST_CONTROLLER_NAME);
  //
  //   ASSERT_EQ(
  //     std::future_status::timeout,
  //     unload_future.wait_for(std::chrono::milliseconds(100))) <<
  //     "unload_controller should be blocking until next update cycle";
  //   ControllerManagerRunner cm_runner(this);
  //   EXPECT_EQ(
  //     controller_interface::return_type::OK,
  //     unload_future.get()
  //   );
  //
  //   EXPECT_EQ(
  //     lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
  //     test_controller->get_current_state().id());
  //   EXPECT_EQ(1, test_controller.use_count());
}
