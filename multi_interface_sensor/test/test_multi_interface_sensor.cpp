// Copyright 2020 ros2_control Development Team
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
#include <string>
#include <vector>

#include "multi_interface_sensor/multi_interface_sensor.hpp"

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ::testing;  // NOLINT

class TestMultiInterfaceSensor : public Test
{
};

TEST_F(TestMultiInterfaceSensor, empty_initialized)
{
  multi_interface_sensor::MultiInterfaceSensor sensor;
  EXPECT_TRUE(sensor.get_state_interfaces().empty());
}

TEST_F(TestMultiInterfaceSensor, wrong_initialized)
{
  {
    multi_interface_sensor::MultiInterfaceSensor sensor;
    hardware_interface::components::ComponentInfo sensor_info;
    EXPECT_EQ(
      hardware_interface::return_type::INTERFACE_NOT_PROVIDED, sensor.configure(sensor_info));
  }

  {
    hardware_interface::components::ComponentInfo sensor_info;
    sensor_info.state_interfaces.push_back("position");
    sensor_info.state_interfaces.push_back("position");
    multi_interface_sensor::MultiInterfaceSensor sensor;
    EXPECT_EQ(hardware_interface::return_type::INTERFACE_DUPLICATES, sensor.configure(sensor_info));
  }
}

TEST_F(TestMultiInterfaceSensor, correct_initialized)
{
  {
    hardware_interface::components::ComponentInfo sensor_info;
    sensor_info.state_interfaces.push_back("effort");
    multi_interface_sensor::MultiInterfaceSensor sensor;
    EXPECT_EQ(hardware_interface::return_type::OK, sensor.configure(sensor_info));
    ASSERT_EQ(1u, sensor.get_state_interfaces().size());
    EXPECT_EQ("effort", sensor.get_state_interfaces()[0]);
  }
}

TEST_F(TestMultiInterfaceSensor, getters_and_setters)
{
  multi_interface_sensor::MultiInterfaceSensor sensor;
  hardware_interface::components::ComponentInfo sensor_info;
  sensor_info.state_interfaces.push_back("force_x");
  sensor_info.state_interfaces.push_back("force_y");
  sensor_info.state_interfaces.push_back("force_z");
  sensor_info.state_interfaces.push_back("torque_x");
  sensor_info.state_interfaces.push_back("torque_y");
  sensor_info.state_interfaces.push_back("torque_z");
  ASSERT_EQ(
    hardware_interface::return_type::OK, sensor.configure(sensor_info));
  // default initialize values
  ASSERT_EQ(
    hardware_interface::return_type::OK,
    sensor.set_state({1.34, 5.67, 8.21, 5.63, 5.99, 4.32}));

  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.configure(sensor_info));
  ASSERT_EQ(6u, sensor.get_state_interfaces().size());
  EXPECT_EQ(sensor.get_state_interfaces()[0], "force_x");
  EXPECT_EQ(sensor.get_state_interfaces()[5], "torque_z");
  std::vector<double> input = {5.23, 6.7, 2.5, 3.8, 8.9, 12.3};
  std::vector<double> output;
  std::vector<std::string> interfaces;
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_PROVIDED,
    sensor.get_state(output, interfaces));
  interfaces.push_back("force_y");
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.get_state(output, interfaces));
  ASSERT_EQ(1u, output.size());
  EXPECT_EQ(5.67, output[0]);

  // State getters and setters
  interfaces.clear();
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_PROVIDED,
    sensor.set_state(input, interfaces));
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    sensor.set_state(input, interfaces));
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    sensor.set_state(input, interfaces));
  interfaces.clear();
  interfaces = sensor.get_state_interfaces();
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.set_state(input, interfaces));

  output.clear();
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.get_state(output, interfaces));
  ASSERT_EQ(6u, output.size());
  EXPECT_EQ(5.23, output[0]);

  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_NOT_FOUND,
    sensor.get_state(output, interfaces));

  input.clear();
  EXPECT_EQ(
    hardware_interface::return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL,
    sensor.set_state(input));
  input = {5.23, 6.7, 2.5, 3.8, 8.9, 12.3};
  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.set_state(input));

  EXPECT_EQ(
    hardware_interface::return_type::OK,
    sensor.get_state(output));
  ASSERT_EQ(6u, output.size());
  EXPECT_EQ(12.3, output[5]);
}
