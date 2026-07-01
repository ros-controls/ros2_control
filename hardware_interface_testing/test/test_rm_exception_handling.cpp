// Copyright 2024 ros2_control Development Team
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
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

using hardware_interface::ResourceManager;
using hardware_interface::ResourceManagerParams;
using hardware_interface::return_type;
namespace lifecycle_state_names = hardware_interface::lifecycle_state_names;

// ---------------------------------------------------------------------------
// URDF snippets — these match exactly what TestActuator/Sensor/System expect
// ---------------------------------------------------------------------------

// A single TestActuator component with the joints/interfaces it requires.
// TestActuator::on_init checks joints[0].state_interfaces[1].name != "does_not_exist"
// and needs at least position + velocity state interfaces and position + max_velocity command.
static const char * const ACTUATOR_HW =
  R"(
  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity"/>
    </joint>
  </ros2_control>
)";

// A single TestSensor component.
// TestSensor::on_init checks sensors[0].state_interfaces.size() != 2 (only 1 is OK).
static const char * const SENSOR_HW =
  R"(
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <plugin>test_sensor</plugin>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="velocity"/>
    </sensor>
  </ros2_control>
)";

// A single TestSystem component with the minimal structure it requires.
// TestSystem::on_init checks joints[0].state_interfaces[1].name != "does_not_exist".
// on_export_command_interfaces accesses joints[0].command_interfaces[1] (max_acceleration).
static const char * const SYSTEM_HW =
  R"(
  <ros2_control name="TestSystemHardware" type="system">
    <hardware>
      <plugin>test_system</plugin>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

// ---------------------------------------------------------------------------
// Helper: build a URDF by injecting extra <param> tags into a hw snippet
// ---------------------------------------------------------------------------
static std::string make_urdf(const std::string & hw_snippet, const std::string & extra_params = "")
{
  // If no extra params, just use the snippet as-is
  if (extra_params.empty())
  {
    return std::string(ros2_control_test_assets::urdf_head) + hw_snippet +
           ros2_control_test_assets::urdf_tail;
  }

  // Inject extra params after the <plugin>...</plugin> line
  std::string snippet = hw_snippet;
  const std::string plugin_close = "</plugin>";
  auto pos = snippet.find(plugin_close);
  if (pos != std::string::npos)
  {
    snippet.insert(pos + plugin_close.size(), "\n" + extra_params);
  }
  return std::string(ros2_control_test_assets::urdf_head) + snippet +
         ros2_control_test_assets::urdf_tail;
}

// ---------------------------------------------------------------------------
// Test Fixture
// ---------------------------------------------------------------------------
class TestResourceManagerExceptionHandling : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build a default valid robot description (no failures expected)
    params_.robot_description = std::string(ros2_control_test_assets::urdf_head) + ACTUATOR_HW +
                                SENSOR_HW + SYSTEM_HW + ros2_control_test_assets::urdf_tail;
    params_.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params_.handle_exceptions = true;  // default: swallow exceptions
  }

  ResourceManagerParams params_;
};

// ---------------------------------------------------------------------------
// 1. on_init exception — Actuator
//    Covers: ResourceStorage::initialize_hardware catch(std::exception) and catch(...)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, actuator_init_standard_exception)
{
  params_.robot_description =
    make_urdf(ACTUATOR_HW, "      <param name=\"throw_on_on_init\">true</param>");

  // handle_exceptions = true => absorb, no throw at RM level
  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  // handle_exceptions = false => propagate
  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, actuator_init_unknown_exception)
{
  params_.robot_description =
    make_urdf(ACTUATOR_HW, "      <param name=\"throw_unknown_on_on_init\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  // RM's catch(...) re-throws — EXPECT_ANY_THROW since it is not std::exception
  EXPECT_ANY_THROW({ ResourceManager rm(params_, true); });
}

// ---------------------------------------------------------------------------
// 2. on_init exception — Sensor
//    Covers: the sensor-branch of initialize_hardware
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, sensor_init_standard_exception)
{
  params_.robot_description =
    make_urdf(SENSOR_HW, "      <param name=\"throw_on_on_init\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, std::runtime_error);
}

// ---------------------------------------------------------------------------
// 3. on_init exception — System
//    Covers: the system-branch of initialize_hardware
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_init_standard_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_init\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, std::runtime_error);
}

// ---------------------------------------------------------------------------
// 4. export_state_interfaces exception
//    Covers: import_state_interfaces catch(std::exception) — newly added block
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_export_state_interfaces_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_export_state_interfaces\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_export_state_interfaces_unknown_exception)
{
  params_.robot_description = make_urdf(
    SYSTEM_HW, "      <param name=\"throw_unknown_on_export_state_interfaces\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_ANY_THROW({ ResourceManager rm(params_, true); });
}

// ---------------------------------------------------------------------------
// 5. export_command_interfaces exception
//    Covers: import_command_interfaces catch(std::exception) and catch(...)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_export_command_interfaces_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_export_command_interfaces\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_export_command_interfaces_unknown_exception)
{
  params_.robot_description = make_urdf(
    SYSTEM_HW, "      <param name=\"throw_unknown_on_export_command_interfaces\">true</param>");

  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  params_.handle_exceptions = false;
  EXPECT_ANY_THROW({ ResourceManager rm(params_, true); });
}

// ---------------------------------------------------------------------------
// 6. Lifecycle transition exceptions: configure, activate, deactivate
//    Covers: configure_hardware, activate_hardware, deactivate_hardware catch blocks
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_configure_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_configure\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  // set_component_state should return ERROR (not throw) when handle_exceptions=true
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", inactive_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  EXPECT_THROW(
    rm_throwing.set_component_state("TestSystemHardware", inactive_state), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_activate_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_activate\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);

  // Need to configure first before activating
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
  EXPECT_EQ(rm_handled.set_component_state("TestSystemHardware", active_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_THROW(
    rm_throwing.set_component_state("TestSystemHardware", active_state), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_deactivate_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_deactivate\">true</param>");

  // First boot the component to ACTIVE, then try to deactivate — that should throw
  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);

  // Deactivate back to INACTIVE
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", inactive_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_THROW(
    rm_throwing.set_component_state("TestSystemHardware", inactive_state), std::runtime_error);
}

// ---------------------------------------------------------------------------
// 7. read / write exceptions
//    Covers: read_hardware and write_hardware catch blocks in ResourceManager
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_read_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_read\">true</param>");

  // handle_exceptions = true: read() should return ERROR but not throw
  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);

  EXPECT_NO_THROW(
    rm_handled.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));

  // handle_exceptions = false: read() should throw
  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);

  EXPECT_THROW(
    rm_throwing.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)),
    std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_write_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_write\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);

  EXPECT_NO_THROW(
    rm_handled.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);

  EXPECT_THROW(
    rm_throwing.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)),
    std::runtime_error);
}

// ---------------------------------------------------------------------------
// 9. Unknown exceptions for configure / activate / deactivate
//    Covers: configure_hardware catch(...), activate_hardware catch(...),
//            deactivate_hardware catch(...)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_configure_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_on_configure\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", inactive_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  EXPECT_ANY_THROW(rm_throwing.set_component_state("TestSystemHardware", inactive_state));
}

TEST_F(TestResourceManagerExceptionHandling, system_activate_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_on_activate\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_EQ(rm_handled.set_component_state("TestSystemHardware", active_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_ANY_THROW(rm_throwing.set_component_state("TestSystemHardware", active_state));
}

TEST_F(TestResourceManagerExceptionHandling, system_deactivate_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_on_deactivate\">true</param>");

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", inactive_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_ANY_THROW(rm_throwing.set_component_state("TestSystemHardware", inactive_state));
}

// ---------------------------------------------------------------------------
// 10. cleanup_hardware exceptions (requires INACTIVE state first)
//     Covers: cleanup_hardware catch(std::exception) and catch(...)
//     State transition: UNCONFIGURED -> configure -> INACTIVE
//                       INACTIVE -> cleanup -> UNCONFIGURED
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_cleanup_standard_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_cleanup\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State unconfigured_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_state_names::UNCONFIGURED);

  // handle_exceptions = true: cleanup() throws, RM catches and returns ERROR
  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", unconfigured_state), return_type::ERROR);

  // handle_exceptions = false: cleanup() throws, RM re-throws it
  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_THROW(
    rm_throwing.set_component_state("TestSystemHardware", unconfigured_state), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_cleanup_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_on_cleanup\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State unconfigured_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_state_names::UNCONFIGURED);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", unconfigured_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  EXPECT_ANY_THROW(rm_throwing.set_component_state("TestSystemHardware", unconfigured_state));
}

// ---------------------------------------------------------------------------
// 11. shutdown_hardware exceptions
//     Covers: shutdown_hardware catch(std::exception) and catch(...)
//     State transition: UNCONFIGURED -> FINALIZED (calls shutdown_hardware)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_shutdown_standard_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_on_shutdown\">true</param>");

  // After loading, hardware is in UNCONFIGURED. Moving to FINALIZED calls shutdown.
  rclcpp_lifecycle::State finalized_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", finalized_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  EXPECT_THROW(
    rm_throwing.set_component_state("TestSystemHardware", finalized_state), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, system_shutdown_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_on_shutdown\">true</param>");

  rclcpp_lifecycle::State finalized_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  EXPECT_EQ(
    rm_handled.set_component_state("TestSystemHardware", finalized_state), return_type::ERROR);

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  EXPECT_ANY_THROW(rm_throwing.set_component_state("TestSystemHardware", finalized_state));
}

// ---------------------------------------------------------------------------
// 12. load_hardware pluginlib exception
//     Covers: load_hardware catch(pluginlib::PluginlibException)
//     Triggered naturally by an invalid plugin name in the URDF — no mock needed.
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, load_hardware_pluginlib_exception)
{
  // A deliberately invalid plugin name triggers PluginlibException during dlopen.
  const char * const BAD_PLUGIN_HW =
    R"(
  <ros2_control name="BadPluginHardware" type="system">
    <hardware>
      <plugin>this_plugin_does_not_exist</plugin>
    </hardware>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <command_interface name="max_acceleration"/>
    </joint>
    <joint name="joint3">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="configuration">
      <command_interface name="max_tcp_jerk"/>
      <state_interface name="max_tcp_jerk"/>
    </gpio>
  </ros2_control>
)";

  params_.robot_description = std::string(ros2_control_test_assets::urdf_head) + BAD_PLUGIN_HW +
                              ros2_control_test_assets::urdf_tail;

  // handle_exceptions = true: error logged, no throw — component simply not loaded
  params_.handle_exceptions = true;
  EXPECT_NO_THROW({ ResourceManager rm(params_, true); });

  // handle_exceptions = false: PluginlibException propagates from load_hardware
  params_.handle_exceptions = false;
  EXPECT_THROW({ ResourceManager rm(params_, true); }, pluginlib::PluginlibException);
}

// ---------------------------------------------------------------------------
// 13. prepare_command_mode_switch exceptions
//     Covers: call_prepare_mode_switch lambda catch(std::exception) and catch(...)
//     Component must be in INACTIVE or ACTIVE before calling prepare_command_mode_switch.
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, prepare_command_mode_switch_standard_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_prepare_command_mode_switch\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  // handle_exceptions = true: exception caught, returns false
  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_FALSE(rm_handled.prepare_command_mode_switch({"joint2/velocity"}, {}));

  // handle_exceptions = false: exception re-thrown
  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_THROW(
    rm_throwing.prepare_command_mode_switch({"joint2/velocity"}, {}), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, prepare_command_mode_switch_unknown_exception)
{
  params_.robot_description = make_urdf(
    SYSTEM_HW, "      <param name=\"throw_unknown_on_prepare_command_mode_switch\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_FALSE(rm_handled.prepare_command_mode_switch({"joint2/velocity"}, {}));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_ANY_THROW(rm_throwing.prepare_command_mode_switch({"joint2/velocity"}, {}));
}

// ---------------------------------------------------------------------------
// 14. perform_command_mode_switch exceptions
//     Covers: call_perform_mode_switch lambda catch(std::exception) and catch(...)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, perform_command_mode_switch_standard_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_on_perform_command_mode_switch\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_FALSE(rm_handled.perform_command_mode_switch({"joint2/velocity"}, {}));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_THROW(
    rm_throwing.perform_command_mode_switch({"joint2/velocity"}, {}), std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, perform_command_mode_switch_unknown_exception)
{
  params_.robot_description = make_urdf(
    SYSTEM_HW, "      <param name=\"throw_unknown_on_perform_command_mode_switch\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_FALSE(rm_handled.perform_command_mode_switch({"joint2/velocity"}, {}));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_ANY_THROW(rm_throwing.perform_command_mode_switch({"joint2/velocity"}, {}));
}

// ---------------------------------------------------------------------------
// 15. Unknown exceptions during read / write
//     Covers: read catch(...) and write catch(...)
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, system_read_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_read\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_NO_THROW(
    rm_handled.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_ANY_THROW(
    rm_throwing.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));
}

TEST_F(TestResourceManagerExceptionHandling, system_write_unknown_exception)
{
  params_.robot_description =
    make_urdf(SYSTEM_HW, "      <param name=\"throw_unknown_on_write\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = true;
  ResourceManager rm_handled(params_, true);
  rm_handled.set_component_state("TestSystemHardware", inactive_state);
  rm_handled.set_component_state("TestSystemHardware", active_state);
  EXPECT_NO_THROW(
    rm_handled.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));

  params_.handle_exceptions = false;
  ResourceManager rm_throwing(params_, true);
  rm_throwing.set_component_state("TestSystemHardware", inactive_state);
  rm_throwing.set_component_state("TestSystemHardware", active_state);
  EXPECT_ANY_THROW(
    rm_throwing.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));
}

// ---------------------------------------------------------------------------
// 16. Actuator read / write exceptions
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, actuator_read_write_exception)
{
  params_.robot_description = make_urdf(
    ACTUATOR_HW,
    "      <param name=\"throw_on_read\">true</param>\n"
    "      <param name=\"throw_on_write\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = false;
  ResourceManager rm(params_, true);
  rm.set_component_state("TestActuatorHardware", inactive_state);
  rm.set_component_state("TestActuatorHardware", active_state);

  EXPECT_THROW(
    rm.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)),
    std::runtime_error);
  EXPECT_THROW(
    rm.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)),
    std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, actuator_read_write_unknown_exception)
{
  params_.robot_description = make_urdf(
    ACTUATOR_HW,
    "      <param name=\"throw_unknown_on_read\">true</param>\n"
    "      <param name=\"throw_unknown_on_write\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = false;
  ResourceManager rm(params_, true);
  rm.set_component_state("TestActuatorHardware", inactive_state);
  rm.set_component_state("TestActuatorHardware", active_state);

  EXPECT_ANY_THROW(
    rm.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));
  EXPECT_ANY_THROW(
    rm.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));
}

// ---------------------------------------------------------------------------
// 17. Sensor read exception
// ---------------------------------------------------------------------------
TEST_F(TestResourceManagerExceptionHandling, sensor_read_exception)
{
  params_.robot_description =
    make_urdf(SENSOR_HW, "      <param name=\"throw_on_read\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = false;
  ResourceManager rm(params_, true);
  rm.set_component_state("TestSensorHardware", inactive_state);
  rm.set_component_state("TestSensorHardware", active_state);

  EXPECT_THROW(
    rm.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)),
    std::runtime_error);
}

TEST_F(TestResourceManagerExceptionHandling, sensor_read_unknown_exception)
{
  params_.robot_description =
    make_urdf(SENSOR_HW, "      <param name=\"throw_unknown_on_read\">true</param>");

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);

  params_.handle_exceptions = false;
  ResourceManager rm(params_, true);
  rm.set_component_state("TestSensorHardware", inactive_state);
  rm.set_component_state("TestSensorHardware", active_state);

  EXPECT_ANY_THROW(
    rm.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01)));
}
