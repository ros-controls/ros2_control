// Copyright 2025 Aarav Gupta
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

#include "test_magnetic_field_sensor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <vector>
#include "sensor_msgs/msg/magnetic_field.hpp"

void MagneticFieldSensorTest::TearDown() { magnetic_field_sensor_.reset(nullptr); }

TEST_F(MagneticFieldSensorTest, validate_all)
{
  // Create the magnetic field sensor
  magnetic_field_sensor_ = std::make_unique<TestableMagneticFieldSensor>(sensor_name_);

  // validate the component name
  ASSERT_EQ(magnetic_field_sensor_->name_, sensor_name_);

  // Validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(magnetic_field_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.capacity(), size_);

  // Validate the default interface_names_
  ASSERT_TRUE(
    std::equal(
      magnetic_field_sensor_->interface_names_.begin(),
      magnetic_field_sensor_->interface_names_.end(), full_interface_names_.begin(),
      full_interface_names_.end()));

  // Get the interface names
  std::vector<std::string> interface_names = magnetic_field_sensor_->get_state_interface_names();

  // Assign values
  auto magnetic_field_x = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[0]);
  std::ignore = magnetic_field_x->set_value(magnetic_field_values_[0]);
  auto magnetic_field_y = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[1]);
  std::ignore = magnetic_field_y->set_value(magnetic_field_values_[1]);
  auto magnetic_field_z = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[2]);
  std::ignore = magnetic_field_z->set_value(magnetic_field_values_[2]);

  // Create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(10);

  // Insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(magnetic_field_y);
  temp_state_interfaces.emplace_back(magnetic_field_z);
  temp_state_interfaces.emplace_back(magnetic_field_x);

  // Now call the function to make them in order like interface_names
  magnetic_field_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // Validate the count of state_interfaces_
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.size(), size_);

  // Validate get_values_as_message()
  sensor_msgs::msg::MagneticField temp_message;
  ASSERT_TRUE(magnetic_field_sensor_->get_values_as_message(temp_message));
  ASSERT_EQ(temp_message.magnetic_field.x, magnetic_field_values_[0]);
  ASSERT_EQ(temp_message.magnetic_field.y, magnetic_field_values_[1]);
  ASSERT_EQ(temp_message.magnetic_field.z, magnetic_field_values_[2]);

  // release the state_interfaces_
  magnetic_field_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(magnetic_field_sensor_->state_interfaces_.size(), 0u);
}

TEST_F(MagneticFieldSensorTest, axes_start_as_nan_and_keep_the_last_reading)
{
  semantic_components::MagneticFieldSensor sensor(sensor_name_);
  std::array<hardware_interface::StateInterface::SharedPtr, 3> states;
  std::vector<hardware_interface::LoanedStateInterface> interfaces;
  for (auto i = 0u; i < states.size(); ++i)
  {
    states[i] = std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, magnetic_field_interface_names_[i]);
    ASSERT_TRUE(states[i]->set_value(magnetic_field_values_[i]));
    interfaces.emplace_back(states[i]);
  }
  ASSERT_TRUE(sensor.assign_loaned_state_interfaces(interfaces));

  sensor_msgs::msg::MagneticField message;
  {
    std::scoped_lock lock(states[0]->get_mutex(), states[1]->get_mutex(), states[2]->get_mutex());
    // Keep the interfaces locked while another thread exhausts the nonblocking reads.
    EXPECT_TRUE(
      std::async(std::launch::async, [&] { return sensor.get_values_as_message(message); }).get());
    EXPECT_TRUE(std::isnan(message.magnetic_field.x));
    EXPECT_TRUE(std::isnan(message.magnetic_field.y));
    EXPECT_TRUE(std::isnan(message.magnetic_field.z));
  }
  EXPECT_TRUE(sensor.get_values_as_message(message));
  EXPECT_DOUBLE_EQ(message.magnetic_field.x, magnetic_field_values_[0]);
  EXPECT_DOUBLE_EQ(message.magnetic_field.y, magnetic_field_values_[1]);
  EXPECT_DOUBLE_EQ(message.magnetic_field.z, magnetic_field_values_[2]);

  auto last_readings = magnetic_field_values_;
  for (auto unavailable_axis = 0u; unavailable_axis < states.size(); ++unavailable_axis)
  {
    SCOPED_TRACE(unavailable_axis);
    std::array<double, 3> values;
    for (auto i = 0u; i < states.size(); ++i)
    {
      values[i] = 10.0 * (unavailable_axis + 1) + i;
      ASSERT_TRUE(states[i]->set_value(values[i]));
    }

    {
      std::unique_lock<std::shared_mutex> lock(states[unavailable_axis]->get_mutex());
      EXPECT_TRUE(
        std::async(std::launch::async, [&] { return sensor.get_values_as_message(message); })
          .get());
      const std::array<double, 3> observed{
        {message.magnetic_field.x, message.magnetic_field.y, message.magnetic_field.z}};
      for (auto i = 0u; i < states.size(); ++i)
      {
        const double expected = i == unavailable_axis ? last_readings[i] : values[i];
        EXPECT_DOUBLE_EQ(observed[i], expected);
      }
    }

    EXPECT_TRUE(sensor.get_values_as_message(message));
    EXPECT_DOUBLE_EQ(message.magnetic_field.x, values[0]);
    EXPECT_DOUBLE_EQ(message.magnetic_field.y, values[1]);
    EXPECT_DOUBLE_EQ(message.magnetic_field.z, values[2]);
    last_readings = values;
  }
}
