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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_test_common.hpp"

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/msg/hardware_component_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/parameter.hpp"

using ::testing::_;
using ::testing::Return;

using hardware_interface::lifecycle_state_names::ACTIVE;
using hardware_interface::lifecycle_state_names::FINALIZED;
using hardware_interface::lifecycle_state_names::INACTIVE;
using hardware_interface::lifecycle_state_names::UNCONFIGURED;
using hardware_interface::lifecycle_state_names::UNKNOWN;

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

using namespace std::chrono_literals;  // NOLINT

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
      "hardware_components_initial_state.unconfigured",
      std::vector<std::string>({TEST_SYSTEM_HARDWARE_NAME})));
    cm_->set_parameter(rclcpp::Parameter(
      "hardware_components_initial_state.inactive",
      std::vector<std::string>({TEST_SENSOR_HARDWARE_NAME})));

    std::string robot_description = "";
    cm_->get_parameter("robot_description", robot_description);
    if (robot_description.empty())
    {
      throw std::runtime_error(
        "Unable to initialize resource manager, no robot description found.");
    }

    auto msg = std_msgs::msg::String();
    msg.data = robot_description;
    cm_->robot_description_callback(msg);

    SetUpSrvsCMExecutor();
  }

  void check_component_fileds(
    const controller_manager_msgs::msg::HardwareComponentState & component,
    const std::string & name, const std::string & type, const std::string & class_type,
    const uint8_t state_id, const std::string & state_label)
  {
    EXPECT_EQ(component.name, name) << "Component has unexpected name.";
    EXPECT_EQ(component.type, type)
      << "Component " << name << " from plugin " << class_type << " has wrong type.";
    EXPECT_EQ(component.class_type, class_type)
      << "Component " << name << " (" << type << ") has unexpected class_type.";
    EXPECT_EQ(component.state.id, state_id)
      << "Component " << name << " (" << type << ") from plugin " << class_type
      << " has wrong state_id.";
    EXPECT_EQ(component.state.label, state_label)
      << "Component " << name << " (" << type << ") from plugin " << class_type
      << " has wrong state_label.";
  }

  void list_hardware_components_and_check(
    const std::vector<uint8_t> & hw_state_ids, const std::vector<std::string> & hw_state_labels,
    const std::vector<std::vector<std::vector<bool>>> & hw_itfs_available_status,
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
        const std::vector<std::string> & interface_names,
        const std::vector<bool> is_available_status, const std::vector<bool> is_claimed_status)
    {
      for (auto i = 0ul; i < interfaces.size(); ++i)
      {
        auto it = std::find(interface_names.begin(), interface_names.end(), interfaces[i].name);
        EXPECT_NE(it, interface_names.end());
        EXPECT_EQ(interfaces[i].is_available, is_available_status[i])
          << "At " << interfaces[i].name;
        EXPECT_EQ(interfaces[i].is_claimed, is_claimed_status[i]) << "At " << interfaces[i].name;
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
          hw_itfs_available_status[0][0], hw_itfs_claimed_status[0][0]);
        check_interfaces(
          component.state_interfaces, TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
          hw_itfs_available_status[0][1], hw_itfs_claimed_status[0][1]);
      }
      if (component.name == TEST_SENSOR_HARDWARE_NAME)
      {
        check_component_fileds(
          component, TEST_SENSOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_TYPE,
          TEST_SENSOR_HARDWARE_CLASS_TYPE, hw_state_ids[1], hw_state_labels[1]);
        check_interfaces(
          component.command_interfaces, TEST_SENSOR_HARDWARE_COMMAND_INTERFACES,
          hw_itfs_available_status[1][0], hw_itfs_claimed_status[1][0]);
        check_interfaces(
          component.state_interfaces, TEST_SENSOR_HARDWARE_STATE_INTERFACES,
          hw_itfs_available_status[1][1], hw_itfs_claimed_status[1][1]);
      }
      if (component.name == TEST_SYSTEM_HARDWARE_NAME)
      {
        check_component_fileds(
          component, TEST_SYSTEM_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_TYPE,
          TEST_SYSTEM_HARDWARE_CLASS_TYPE, hw_state_ids[2], hw_state_labels[2]);
        check_interfaces(
          component.command_interfaces, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
          hw_itfs_available_status[2][0], hw_itfs_claimed_status[2][0]);
        check_interfaces(
          component.state_interfaces, TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
          hw_itfs_available_status[2][1], hw_itfs_claimed_status[2][1]);
      }
    }
  }

  bool set_hardware_component_state(
    const std::string & hardware_name, const uint8_t target_state_id,
    const std::string & target_state_label)
  {
    rclcpp::executors::SingleThreadedExecutor srv_executor;
    rclcpp::Node::SharedPtr mha_srv_node = std::make_shared<rclcpp::Node>("mha_srv_client");
    srv_executor.add_node(mha_srv_node);
    rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr mha_client =
      mha_srv_node->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
        std::string(TEST_CM_NAME) + "/set_hardware_component_state");
    auto request =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();

    request->name = hardware_name;
    request->target_state.id = target_state_id;
    request->target_state.label = target_state_label;

    auto result = call_service_and_wait(*mha_client, request, srv_executor);
    return result->ok;
  }
};

TEST_F(TestControllerManagerHWManagementSrvs, list_hardware_components)
{
  // Default status after start:
  // checks if "configure_components_on_start" and "activate_components_on_start" are correctly read

  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_UNCONFIGURED}),
    std::vector<std::string>({ACTIVE, INACTIVE, UNCONFIGURED}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},  // actuator
      {{}, {true}},                        // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));
}

// TODO(destogl): Add tests for testing controller starting/stopping depending on hw state
TEST_F(TestControllerManagerHWManagementSrvs, selective_activate_deactivate_components_set_state)
{
  using lifecycle_msgs::msg::State;

  // Default status after start
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_UNCONFIGURED}),
    std::vector<std::string>({ACTIVE, INACTIVE, UNCONFIGURED}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},  // actuator
      {{}, {true}},                        // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));

  // Activate sensor
  set_hardware_component_state(TEST_SENSOR_HARDWARE_NAME, State::PRIMARY_STATE_ACTIVE, "");
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_UNCONFIGURED}),
    std::vector<std::string>({ACTIVE, ACTIVE, UNCONFIGURED}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},  // actuator
      {{}, {true}},                        // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));

  // Activate system directly - it should do configure automatically; ID is determined from label
  set_hardware_component_state(TEST_SYSTEM_HARDWARE_NAME, 0, ACTIVE);
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({ACTIVE, ACTIVE, ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},                                      // actuator
      {{}, {true}},                                                            // sensor
      {{true, true, true, true}, {true, true, true, true, true, true, true}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));

  // Deactivate actuator
  set_hardware_component_state(TEST_ACTUATOR_HARDWARE_NAME, 0, INACTIVE);
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({INACTIVE, ACTIVE, ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},                                      // actuator
      {{}, {true}},                                                            // sensor
      {{true, true, true, true}, {true, true, true, true, true, true, true}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));

  // Double activate system
  set_hardware_component_state(TEST_SYSTEM_HARDWARE_NAME, LFC_STATE::PRIMARY_STATE_ACTIVE, ACTIVE);
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({INACTIVE, ACTIVE, ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},                                      // actuator
      {{}, {true}},                                                            // sensor
      {{true, true, true, true}, {true, true, true, true, true, true, true}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));

  // Set sensor to UNCONFIGURED - inactive and cleanup transitions has to be automatically done
  set_hardware_component_state(
    TEST_SENSOR_HARDWARE_NAME, LFC_STATE::PRIMARY_STATE_UNCONFIGURED, UNCONFIGURED);
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_INACTIVE, LFC_STATE::PRIMARY_STATE_UNCONFIGURED,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({INACTIVE, UNCONFIGURED, ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},                                      // actuator
      {{}, {false}},                                                           // sensor
      {{true, true, true, true}, {true, true, true, true, true, true, true}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));
}

class TestControllerManagerHWManagementSrvsWithoutParams
: public TestControllerManagerHWManagementSrvs
{
public:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(), executor_, TEST_CM_NAME);
    run_updater_ = false;

    // TODO(destogl): separate this to init_tests method where parameter can be set for each test
    // separately
    cm_->set_parameter(
      rclcpp::Parameter("robot_description", ros2_control_test_assets::minimal_robot_urdf));

    std::string robot_description = "";
    cm_->get_parameter("robot_description", robot_description);
    if (robot_description.empty())
    {
      throw std::runtime_error(
        "Unable to initialize resource manager, no robot description found.");
    }

    auto msg = std_msgs::msg::String();
    msg.data = robot_description;
    cm_->robot_description_callback(msg);

    SetUpSrvsCMExecutor();
  }
};

TEST_F(TestControllerManagerHWManagementSrvsWithoutParams, test_default_activation_of_all_hardware)
{
  // "configure_components_on_start" and "activate_components_on_start" are not set (empty)
  // therefore all hardware components are active
  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_ACTIVE,
       LFC_STATE::PRIMARY_STATE_ACTIVE}),
    std::vector<std::string>({ACTIVE, ACTIVE, ACTIVE}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},                                      // actuator
      {{}, {true}},                                                            // sensor
      {{true, true, true, true}, {true, true, true, true, true, true, true}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));
}

// BEGIN: Deprecated parameters
class TestControllerManagerHWManagementSrvsOldParameters
: public TestControllerManagerHWManagementSrvs
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
      "activate_components_on_start", std::vector<std::string>({TEST_ACTUATOR_HARDWARE_NAME})));
    cm_->set_parameter(rclcpp::Parameter(
      "configure_components_on_start", std::vector<std::string>({TEST_SENSOR_HARDWARE_NAME})));

    std::string robot_description = "";
    cm_->get_parameter("robot_description", robot_description);
    if (robot_description.empty())
    {
      throw std::runtime_error(
        "Unable to initialize resource manager, no robot description found.");
    }

    auto msg = std_msgs::msg::String();
    msg.data = robot_description;
    cm_->robot_description_callback(msg);

    SetUpSrvsCMExecutor();
  }
};

TEST_F(TestControllerManagerHWManagementSrvsOldParameters, list_hardware_components)
{
  // Default status after start:
  // checks if "configure_components_on_start" and "activate_components_on_start" are correctly read

  list_hardware_components_and_check(
    // actuator, sensor, system
    std::vector<uint8_t>(
      {LFC_STATE::PRIMARY_STATE_ACTIVE, LFC_STATE::PRIMARY_STATE_INACTIVE,
       LFC_STATE::PRIMARY_STATE_UNCONFIGURED}),
    std::vector<std::string>({ACTIVE, INACTIVE, UNCONFIGURED}),
    std::vector<std::vector<std::vector<bool>>>({
      // is available
      {{true, true}, {true, true, true}},  // actuator
      {{}, {true}},                        // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }),
    std::vector<std::vector<std::vector<bool>>>({
      // is claimed
      {{false, false}, {false, false, false}},  // actuator
      {{}, {false}},                            // sensor
      {{false, false, false, false}, {false, false, false, false, false, false, false}},  // system
    }));
}
// END: Deprecated parameters
