// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_emergency_stop_signal.hpp"
#include "hardware_interface/types/hardware_interface_error_signals.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_warning_signals.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_components.hpp"
// Values to send over command interface to trigger error in write and read methods

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
constexpr unsigned int TRIGGER_READ_WRITE_ERROR_CALLS = 10000;
const auto emergency_stop_signal_size = 1;
const auto warnig_signals_size = 2;
const auto error_signals_size = 2;
const auto report_signals_size =
  emergency_stop_signal_size + warnig_signals_size + error_signals_size;
}  // namespace

using namespace ::testing;  // NOLINT

namespace test_components
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DummyActuatorDefault : public hardware_interface::ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    // We hardcode the info
    if (
      hardware_interface::ActuatorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    set_state("joint1/position", 0.0);
    set_state("joint1/velocity", 0.0);
    set_emergency_stop(0.0);
    set_error_code(0.0);
    set_error_message(0.0);
    set_warning_code(0.0);
    set_warning_message(0.0);

    if (recoverable_error_happened_)
    {
      set_command("joint1/velocity", 0.0);
    }

    read_calls_ = 0;
    write_calls_ = 0;

    return CallbackReturn::SUCCESS;
  }

  std::string get_name() const override { return "DummyActuatorDefault"; }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    ++read_calls_;
    if (read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS)
    {
      set_emergency_stop(1.0);
      set_error_code(1.0);
      set_error_message(1.0);
      set_warning_code(1.0);
      set_warning_message(1.0);
      return hardware_interface::return_type::ERROR;
    }

    // no-op, state is getting propagated within write.
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    ++write_calls_;
    if (write_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS)
    {
      set_emergency_stop(1.0);
      set_error_code(1.0);
      set_error_message(1.0);
      set_warning_code(1.0);
      set_warning_message(1.0);
      return hardware_interface::return_type::ERROR;
    }
    auto position_state = get_state("joint1/position");
    set_state("joint1/position", position_state + get_command("joint1/velocity"));
    set_state("joint1/velocity", get_command("joint1/velocity"));

    return hardware_interface::return_type::OK;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    set_state("joint1/velocity", 0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (!recoverable_error_happened_)
    {
      recoverable_error_happened_ = true;
      return CallbackReturn::SUCCESS;
    }
    else
    {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::FAILURE;
  }

private:
  // Helper variables to initiate error on read
  unsigned int read_calls_ = 0;
  unsigned int write_calls_ = 0;
  bool recoverable_error_happened_ = false;
};

class DummySensorDefault : public hardware_interface::SensorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (
      hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // We hardcode the info
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    set_error_code(0.0);
    set_error_message(0.0);
    set_warning_code(0.0);
    set_warning_message(0.0);
    read_calls_ = 0;
    return CallbackReturn::SUCCESS;
  }

  std::string get_name() const override { return "DummySensorDefault"; }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    ++read_calls_;
    if (read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS)
    {
      set_error_code(1.0);
      set_error_message(1.0);
      set_warning_code(1.0);
      set_warning_message(1.0);
      return hardware_interface::return_type::ERROR;
    }

    // no-op, static value
    set_state("joint1/voltage", voltage_level_hw_value_);
    return hardware_interface::return_type::OK;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (!recoverable_error_happened_)
    {
      recoverable_error_happened_ = true;
      return CallbackReturn::SUCCESS;
    }
    else
    {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::FAILURE;
  }

private:
  double voltage_level_hw_value_ = 0x666;

  // Helper variables to initiate error on read
  int read_calls_ = 0;
  bool recoverable_error_happened_ = false;
};

class DummySystemDefault : public hardware_interface::SystemInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    // We hardcode the info
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto i = 0ul; i < 3; ++i)
    {
      set_state(position_states_[i], 0.0);
      set_state(velocity_states_[i], 0.0);
    }
    set_emergency_stop(0.0);
    set_error_code(0.0);
    set_error_message(0.0);
    set_warning_code(0.0);
    set_warning_message(0.0);
    // reset command only if error is initiated
    if (recoverable_error_happened_)
    {
      for (auto i = 0ul; i < 3; ++i)
      {
        set_command(velocity_commands_[i], 0.0);
      }
    }

    read_calls_ = 0;
    write_calls_ = 0;

    return CallbackReturn::SUCCESS;
  }

  std::string get_name() const override { return "DummySystemDefault"; }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    ++read_calls_;
    if (read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS)
    {
      set_emergency_stop(1.0);
      set_error_code(1.0);
      set_error_message(1.0);
      set_warning_code(1.0);
      set_warning_message(1.0);
      return hardware_interface::return_type::ERROR;
    }

    // no-op, state is getting propagated within write.
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    ++write_calls_;
    if (write_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS)
    {
      set_emergency_stop(1.0);
      set_error_code(1.0);
      set_error_message(1.0);
      set_warning_code(1.0);
      set_warning_message(1.0);
      return hardware_interface::return_type::ERROR;
    }

    for (auto i = 0; i < 3; ++i)
    {
      auto current_pos = get_state(position_states_[i]);
      set_state(position_states_[i], current_pos + get_command(velocity_commands_[i]));
      set_state(velocity_states_[i], get_command(velocity_commands_[i]));
    }
    return hardware_interface::return_type::OK;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (const auto & velocity_state : velocity_states_)
    {
      set_state(velocity_state, 0.0);
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (!recoverable_error_happened_)
    {
      recoverable_error_happened_ = true;
      return CallbackReturn::SUCCESS;
    }
    else
    {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::FAILURE;
  }

private:
  std::vector<std::string> position_states_ = {
    "joint1/position", "joint2/position", "joint3/position"};
  std::vector<std::string> velocity_states_ = {
    "joint1/velocity", "joint2/velocity", "joint3/velocity"};
  std::vector<std::string> velocity_commands_ = {
    "joint1/velocity", "joint2/velocity", "joint3/velocity"};

  // Helper variables to initiate error on read
  unsigned int read_calls_ = 0;
  unsigned int write_calls_ = 0;
  bool recoverable_error_happened_ = false;
};

}  // namespace test_components

TEST(TestComponentInterfaces, dummy_actuator_default)
{
  hardware_interface::Actuator actuator_hw(
    std::make_unique<test_components::DummyActuatorDefault>());
  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_dummy_actuator_only +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo dummy_actuator = control_resources[0];
  auto state = actuator_hw.initialize(dummy_actuator);

  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  const auto state_interface_offset = 2;
  auto state_interfaces = actuator_hw.export_state_interfaces();
  ASSERT_EQ(state_interface_offset + report_signals_size, state_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  // EMERGENCY STOP
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "ActuatorModularJoint1/" + std::string(hardware_interface::EMERGENCY_STOP_SIGNAL));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "ActuatorModularJoint1/" + std::string(hardware_interface::EMERGENCY_STOP_SIGNAL),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::EMERGENCY_STOP_SIGNAL, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("ActuatorModularJoint1", state_interfaces[position]->get_prefix_name());
  }
  // ERROR_SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "ActuatorModularJoint1/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "ActuatorModularJoint1/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("ActuatorModularJoint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces, "ActuatorModularJoint1/" +
                          std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "ActuatorModularJoint1/" +
        std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("ActuatorModularJoint1", state_interfaces[position]->get_prefix_name());
  }
  // WARNING SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "ActuatorModularJoint1/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "ActuatorModularJoint1/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("ActuatorModularJoint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces, "ActuatorModularJoint1/" +
                          std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "ActuatorModularJoint1/" +
        std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("ActuatorModularJoint1", state_interfaces[position]->get_prefix_name());
  }

  auto command_interfaces = actuator_hw.export_command_interfaces();
  ASSERT_EQ(1u, command_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
}

TEST(TestComponentInterfaces, dummy_sensor_default)
{
  hardware_interface::Sensor sensor_hw(std::make_unique<test_components::DummySensorDefault>());

  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_voltage_sensor_only +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo voltage_sensor_res = control_resources[0];
  auto state = sensor_hw.initialize(voltage_sensor_res);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  const auto state_interface_offset = 1;
  auto state_interfaces = sensor_hw.export_state_interfaces();
  ASSERT_EQ(
    state_interface_offset + warnig_signals_size + error_signals_size, state_interfaces.size());
  // check that the normal interfaces get exported as expected
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/voltage");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/voltage", state_interfaces[position]->get_name());
    EXPECT_EQ("voltage", state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
    EXPECT_TRUE(std::isnan(state_interfaces[position]->get_value()));
  }
  // ERROR_SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "SingleJointVoltage/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "SingleJointVoltage/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("SingleJointVoltage", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "SingleJointVoltage/" + std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "SingleJointVoltage/" + std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("SingleJointVoltage", state_interfaces[position]->get_prefix_name());
  }
  // WARNING SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "SingleJointVoltage/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "SingleJointVoltage/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("SingleJointVoltage", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces, "SingleJointVoltage/" +
                          std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "SingleJointVoltage/" +
        std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("SingleJointVoltage", state_interfaces[position]->get_prefix_name());
  }
}

TEST(TestComponentInterfaces, dummy_system_default)
{
  hardware_interface::System system_hw(std::make_unique<test_components::DummySystemDefault>());

  const std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::valid_urdf_ros2_control_dummy_system_robot +
    ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  const hardware_interface::HardwareInfo dummy_system = control_resources[0];
  auto state = system_hw.initialize(dummy_system);
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(hardware_interface::lifecycle_state_names::UNCONFIGURED, state.label());

  const auto state_interface_offset = 6;
  auto state_interfaces = system_hw.export_state_interfaces();
  ASSERT_EQ(state_interface_offset + report_signals_size, state_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint2/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint2/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint3/position");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/position", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(state_interfaces, "joint3/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/velocity", state_interfaces[position]->get_name());
    EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", state_interfaces[position]->get_prefix_name());
  }
  // EMERGENCY STOP
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::EMERGENCY_STOP_SIGNAL));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::EMERGENCY_STOP_SIGNAL),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::EMERGENCY_STOP_SIGNAL, state_interfaces[position]->get_interface_name());
    EXPECT_EQ("RRBotSystemWithGPIO", state_interfaces[position]->get_prefix_name());
  }
  // ERROR_SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::ERROR_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("RRBotSystemWithGPIO", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces, "RRBotSystemWithGPIO/" +
                          std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("RRBotSystemWithGPIO", state_interfaces[position]->get_prefix_name());
  }
  // WARNING SIGNAL
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces,
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "RRBotSystemWithGPIO/" + std::string(hardware_interface::WARNING_SIGNAL_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("RRBotSystemWithGPIO", state_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] = test_components::vector_contains(
      state_interfaces, "RRBotSystemWithGPIO/" +
                          std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME));
    EXPECT_TRUE(contains);
    EXPECT_EQ(
      "RRBotSystemWithGPIO/" +
        std::string(hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME),
      state_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME,
      state_interfaces[position]->get_interface_name());
    EXPECT_EQ("RRBotSystemWithGPIO", state_interfaces[position]->get_prefix_name());
  }

  auto command_interfaces = system_hw.export_command_interfaces();
  ASSERT_EQ(3u, command_interfaces.size());
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint1/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint1/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint1", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint2/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint2/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint2", command_interfaces[position]->get_prefix_name());
  }
  {
    auto [contains, position] =
      test_components::vector_contains(command_interfaces, "joint3/velocity");
    EXPECT_TRUE(contains);
    EXPECT_EQ("joint3/velocity", command_interfaces[position]->get_name());
    EXPECT_EQ(
      hardware_interface::HW_IF_VELOCITY, command_interfaces[position]->get_interface_name());
    EXPECT_EQ("joint3", command_interfaces[position]->get_prefix_name());
  }
}