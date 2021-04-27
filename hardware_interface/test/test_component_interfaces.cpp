// Copyright 2020 ros2_control development team
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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ::testing;  // NOLINT

namespace test_components
{

class DummyActuator : public hardware_interface::ActuatorInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint1", hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint1", hardware_interface::HW_IF_VELOCITY, &velocity_state_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    // We can command in velocity
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "joint1", hardware_interface::HW_IF_VELOCITY, &velocity_command_));

    return command_interfaces;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const override
  {
    return "DummyActuator";
  }

  hardware_interface::status get_status() const override
  {
    return hardware_interface::status::UNKNOWN;
  }

  hardware_interface::return_type read() override
  {
    // no-op, state is getting propagated within write.
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    position_state_ += velocity_command_;
    velocity_state_ = velocity_command_;

    return hardware_interface::return_type::OK;
  }

private:
  double position_state_ = 0.0;
  double velocity_state_ = 0.0;
  double velocity_command_ = 0.0;
};

class DummySensor : public hardware_interface::SensorInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    // We can read some voltage level
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint1", "voltage", &voltage_level_));

    return state_interfaces;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const override
  {
    return "DummySensor";
  }

  hardware_interface::status get_status() const override
  {
    return hardware_interface::status::UNKNOWN;
  }

  hardware_interface::return_type read() override
  {
    // no-op, static value
    return hardware_interface::return_type::OK;
  }

private:
  double voltage_level_ = 0x666;
};

class DummySystem : public hardware_interface::SystemInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint1", hardware_interface::HW_IF_POSITION, &position_state_[0]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint1", hardware_interface::HW_IF_VELOCITY, &velocity_state_[0]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint2", hardware_interface::HW_IF_POSITION, &position_state_[1]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint2", hardware_interface::HW_IF_VELOCITY, &velocity_state_[1]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint3", hardware_interface::HW_IF_POSITION, &position_state_[2]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "joint3", hardware_interface::HW_IF_VELOCITY, &velocity_state_[2]));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    // We can command in velocity
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "joint1", hardware_interface::HW_IF_VELOCITY, &velocity_command_[0]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "joint2", hardware_interface::HW_IF_VELOCITY, &velocity_command_[1]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "joint3", hardware_interface::HW_IF_VELOCITY, &velocity_command_[2]));

    return command_interfaces;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const override
  {
    return "DummySystem";
  }

  hardware_interface::status get_status() const override
  {
    return hardware_interface::status::UNKNOWN;
  }

  hardware_interface::return_type read() override
  {
    // no-op, state is getting propagated within write.
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    for (auto i = 0; i < 3; ++i) {
      position_state_[i] += velocity_command_[0];
      velocity_state_[i] = velocity_command_[0];
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::array<double, 3> position_state_ = {0.0, 0.0, 0.0};
  std::array<double, 3> velocity_state_ = {0.0, 0.0, 0.0};
  std::array<double, 3> velocity_command_ = {0.0, 0.0, 0.0};
};

class DummySystemPreparePerform : public hardware_interface::SystemInterface
{
  // Override the pure virtual functions with default behavior
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    return {};
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const override
  {
    return "DummySystemPreparePerform";
  }

  hardware_interface::status get_status() const override
  {
    return hardware_interface::status::UNKNOWN;
  }

  hardware_interface::return_type read() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    return hardware_interface::return_type::OK;
  }

  // Custom prepare/perform functions
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    // Criteria to test against
    if (start_interfaces.size() != 1) {
      return hardware_interface::return_type::ERROR;
    }
    if (stop_interfaces.size() != 2) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    // Criteria to test against
    if (start_interfaces.size() != 1) {
      return hardware_interface::return_type::ERROR;
    }
    if (stop_interfaces.size() != 2) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
};

}  // namespace test_components

TEST(TestComponentInterfaces, dummy_actuator)
{
  hardware_interface::Actuator actuator_hw(
    std::make_unique<test_components::DummyActuator>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, actuator_hw.configure(mock_hw_info));

  auto state_interfaces = actuator_hw.export_state_interfaces();
  ASSERT_EQ(2u, state_interfaces.size());
  EXPECT_EQ("joint1", state_interfaces[0].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[0].get_interface_name());
  EXPECT_EQ("joint1", state_interfaces[1].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[1].get_interface_name());

  auto command_interfaces = actuator_hw.export_command_interfaces();
  ASSERT_EQ(1u, command_interfaces.size());
  EXPECT_EQ("joint1", command_interfaces[0].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, command_interfaces[0].get_interface_name());

  command_interfaces[0].set_value(1.0);  // velocity
  ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.write());

  for (auto step = 1u; step <= 10; ++step) {
    ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.read());

    ASSERT_EQ(step, state_interfaces[0].get_value());  // position value
    ASSERT_EQ(1u, state_interfaces[1].get_value());  // velocity

    ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.write());
  }

  EXPECT_EQ(
    hardware_interface::return_type::OK,
    actuator_hw.prepare_command_mode_switch({""}, {""}));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    actuator_hw.perform_command_mode_switch({""}, {""}));
}

TEST(TestComponentInterfaces, dummy_sensor)
{
  hardware_interface::Sensor sensor_hw(
    std::make_unique<test_components::DummySensor>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, sensor_hw.configure(mock_hw_info));

  auto state_interfaces = sensor_hw.export_state_interfaces();
  ASSERT_EQ(1u, state_interfaces.size());
  EXPECT_EQ("joint1", state_interfaces[0].get_name());
  EXPECT_EQ("voltage", state_interfaces[0].get_interface_name());
  EXPECT_EQ(0x666, state_interfaces[0].get_value());
}

TEST(TestComponentInterfaces, dummy_system)
{
  hardware_interface::System system_hw(
    std::make_unique<test_components::DummySystem>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, system_hw.configure(mock_hw_info));

  auto state_interfaces = system_hw.export_state_interfaces();
  ASSERT_EQ(6u, state_interfaces.size());
  EXPECT_EQ("joint1", state_interfaces[0].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[0].get_interface_name());
  EXPECT_EQ("joint1", state_interfaces[1].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[1].get_interface_name());
  EXPECT_EQ("joint2", state_interfaces[2].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[2].get_interface_name());
  EXPECT_EQ("joint2", state_interfaces[3].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[3].get_interface_name());
  EXPECT_EQ("joint3", state_interfaces[4].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_POSITION, state_interfaces[4].get_interface_name());
  EXPECT_EQ("joint3", state_interfaces[5].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, state_interfaces[5].get_interface_name());

  auto command_interfaces = system_hw.export_command_interfaces();
  ASSERT_EQ(3u, command_interfaces.size());
  EXPECT_EQ("joint1", command_interfaces[0].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, command_interfaces[0].get_interface_name());
  EXPECT_EQ("joint2", command_interfaces[1].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, command_interfaces[1].get_interface_name());
  EXPECT_EQ("joint3", command_interfaces[2].get_name());
  EXPECT_EQ(hardware_interface::HW_IF_VELOCITY, command_interfaces[2].get_interface_name());

  command_interfaces[0].set_value(1.0);  // velocity
  command_interfaces[1].set_value(1.0);  // velocity
  command_interfaces[2].set_value(1.0);  // velocity
  ASSERT_EQ(hardware_interface::return_type::OK, system_hw.write());

  for (auto step = 1u; step <= 10; ++step) {
    ASSERT_EQ(hardware_interface::return_type::OK, system_hw.read());

    ASSERT_EQ(step, state_interfaces[0].get_value());  // position value
    ASSERT_EQ(1u, state_interfaces[1].get_value());  // velocity
    ASSERT_EQ(step, state_interfaces[2].get_value());  // position value
    ASSERT_EQ(1u, state_interfaces[3].get_value());  // velocity
    ASSERT_EQ(step, state_interfaces[4].get_value());  // position value
    ASSERT_EQ(1u, state_interfaces[5].get_value());  // velocity

    ASSERT_EQ(hardware_interface::return_type::OK, system_hw.write());
  }

  EXPECT_EQ(
    hardware_interface::return_type::OK,
    system_hw.prepare_command_mode_switch({}, {}));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    system_hw.perform_command_mode_switch({}, {}));
}

TEST(TestComponentInterfaces, dummy_command_mode_system)
{
  hardware_interface::System system_hw(
    std::make_unique<test_components::DummySystemPreparePerform>());
  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, system_hw.configure(mock_hw_info));

  std::vector<std::string> one_key = {"joint1/position"};
  std::vector<std::string> two_keys = {"joint1/position", "joint1/velocity"};

  // Only calls with (one_key, two_keys) should return OK
  EXPECT_EQ(
    hardware_interface::return_type::ERROR,
    system_hw.prepare_command_mode_switch(one_key, one_key));
  EXPECT_EQ(
    hardware_interface::return_type::ERROR,
    system_hw.perform_command_mode_switch(one_key, one_key));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    system_hw.prepare_command_mode_switch(one_key, two_keys));
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    system_hw.perform_command_mode_switch(one_key, two_keys));
  EXPECT_EQ(
    hardware_interface::return_type::ERROR,
    system_hw.prepare_command_mode_switch(two_keys, one_key));
  EXPECT_EQ(
    hardware_interface::return_type::ERROR,
    system_hw.perform_command_mode_switch(two_keys, one_key));
}
