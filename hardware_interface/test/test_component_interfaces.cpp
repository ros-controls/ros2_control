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

#include "hardware_interface/components/actuator.hpp"
#include "hardware_interface/components/actuator_interface.hpp"
#include "hardware_interface/components/sensor_interface.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/components/system_interface.hpp"
#include "hardware_interface/components/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ::testing;  // NOLINT

namespace test_components
{

class DummyActuator : public hardware_interface::components::ActuatorInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateHandle> export_state_handles() override
  {
    // We can read a position and a velocity
    std::vector<hardware_interface::StateHandle> state_handles;
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint1", "position", &position_state_));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint1", "velocity", &velocity_state_));

    return state_handles;
  }

  std::vector<hardware_interface::CommandHandle> export_command_handles() override
  {
    // We can command in velocity
    std::vector<hardware_interface::CommandHandle> command_handles;
    command_handles.emplace_back(
      hardware_interface::CommandHandle("joint1", "velocity", &velocity_command_));

    return command_handles;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
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

class DummySensor : public hardware_interface::components::SensorInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateHandle> export_state_handles() override
  {
    // We can read some voltage level
    std::vector<hardware_interface::StateHandle> state_handles;
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint1", "voltage", &voltage_level_));

    return state_handles;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
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

class DummySystem : public hardware_interface::components::SystemInterface
{
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & /* info */) override
  {
    // We hardcode the info
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateHandle> export_state_handles() override
  {
    // We can read a position and a velocity
    std::vector<hardware_interface::StateHandle> state_handles;
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint1", "position", &position_state_[0]));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint1", "velocity", &velocity_state_[0]));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint2", "position", &position_state_[1]));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint2", "velocity", &velocity_state_[1]));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint3", "position", &position_state_[2]));
    state_handles.emplace_back(
      hardware_interface::StateHandle("joint3", "velocity", &velocity_state_[2]));

    return state_handles;
  }

  std::vector<hardware_interface::CommandHandle> export_command_handles() override
  {
    // We can command in velocity
    std::vector<hardware_interface::CommandHandle> command_handles;
    command_handles.emplace_back(
      hardware_interface::CommandHandle("joint1", "velocity", &velocity_command_[0]));
    command_handles.emplace_back(
      hardware_interface::CommandHandle("joint2", "velocity", &velocity_command_[1]));
    command_handles.emplace_back(
      hardware_interface::CommandHandle("joint3", "velocity", &velocity_command_[2]));

    return command_handles;
  }

  hardware_interface::return_type start() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    return hardware_interface::return_type::OK;
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

}  // namespace test_components

TEST(TestComponentInterfaces, dummy_actuator)
{
  hardware_interface::components::Actuator actuator_hw(
    std::make_unique<test_components::DummyActuator>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, actuator_hw.configure(mock_hw_info));

  auto state_handles = actuator_hw.export_state_handles();
  ASSERT_EQ(2u, state_handles.size());
  EXPECT_EQ("joint1", state_handles[0].get_name());
  EXPECT_EQ("position", state_handles[0].get_interface_name());
  EXPECT_EQ("joint1", state_handles[1].get_name());
  EXPECT_EQ("velocity", state_handles[1].get_interface_name());

  auto command_handles = actuator_hw.export_command_handles();
  ASSERT_EQ(1u, command_handles.size());
  EXPECT_EQ("joint1", command_handles[0].get_name());
  EXPECT_EQ("velocity", command_handles[0].get_interface_name());

  command_handles[0].set_value(1.0);  // velocity
  ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.write());

  for (auto step = 1u; step <= 10; ++step) {
    ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.read());

    ASSERT_EQ(step, state_handles[0].get_value());  // position value
    ASSERT_EQ(1u, state_handles[1].get_value());  // velocity

    ASSERT_EQ(hardware_interface::return_type::OK, actuator_hw.write());
  }
}

TEST(TestComponentInterfaces, dummy_sensor)
{
  hardware_interface::components::Sensor sensor_hw(
    std::make_unique<test_components::DummySensor>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, sensor_hw.configure(mock_hw_info));

  auto state_handles = sensor_hw.export_state_handles();
  ASSERT_EQ(1u, state_handles.size());
  EXPECT_EQ("joint1", state_handles[0].get_name());
  EXPECT_EQ("voltage", state_handles[0].get_interface_name());
  EXPECT_EQ(0x666, state_handles[0].get_value());
}

TEST(TestComponentInterfaces, dummy_system)
{
  hardware_interface::components::System system_hw(
    std::make_unique<test_components::DummySystem>());

  hardware_interface::HardwareInfo mock_hw_info{};
  EXPECT_EQ(hardware_interface::return_type::OK, system_hw.configure(mock_hw_info));

  auto state_handles = system_hw.export_state_handles();
  ASSERT_EQ(6u, state_handles.size());
  EXPECT_EQ("joint1", state_handles[0].get_name());
  EXPECT_EQ("position", state_handles[0].get_interface_name());
  EXPECT_EQ("joint1", state_handles[1].get_name());
  EXPECT_EQ("velocity", state_handles[1].get_interface_name());
  EXPECT_EQ("joint2", state_handles[2].get_name());
  EXPECT_EQ("position", state_handles[2].get_interface_name());
  EXPECT_EQ("joint2", state_handles[3].get_name());
  EXPECT_EQ("velocity", state_handles[3].get_interface_name());
  EXPECT_EQ("joint3", state_handles[4].get_name());
  EXPECT_EQ("position", state_handles[4].get_interface_name());
  EXPECT_EQ("joint3", state_handles[5].get_name());
  EXPECT_EQ("velocity", state_handles[5].get_interface_name());

  auto command_handles = system_hw.export_command_handles();
  ASSERT_EQ(3u, command_handles.size());
  EXPECT_EQ("joint1", command_handles[0].get_name());
  EXPECT_EQ("velocity", command_handles[0].get_interface_name());
  EXPECT_EQ("joint2", command_handles[1].get_name());
  EXPECT_EQ("velocity", command_handles[1].get_interface_name());
  EXPECT_EQ("joint3", command_handles[2].get_name());
  EXPECT_EQ("velocity", command_handles[2].get_interface_name());

  command_handles[0].set_value(1.0);  // velocity
  command_handles[1].set_value(1.0);  // velocity
  command_handles[2].set_value(1.0);  // velocity
  ASSERT_EQ(hardware_interface::return_type::OK, system_hw.write());

  for (auto step = 1u; step <= 10; ++step) {
    ASSERT_EQ(hardware_interface::return_type::OK, system_hw.read());

    ASSERT_EQ(step, state_handles[0].get_value());  // position value
    ASSERT_EQ(1u, state_handles[1].get_value());  // velocity
    ASSERT_EQ(step, state_handles[2].get_value());  // position value
    ASSERT_EQ(1u, state_handles[3].get_value());  // velocity
    ASSERT_EQ(step, state_handles[4].get_value());  // position value
    ASSERT_EQ(1u, state_handles[5].get_value());  // velocity

    ASSERT_EQ(hardware_interface::return_type::OK, system_hw.write());
  }
}
