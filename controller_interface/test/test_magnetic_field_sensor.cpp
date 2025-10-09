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

#include <memory>
#include <string>
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
    sensor_name_, magnetic_field_interface_names_[0], &magnetic_field_values_[0]);
  auto magnetic_field_y = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[1], &magnetic_field_values_[1]);
  auto magnetic_field_z = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, magnetic_field_interface_names_[2], &magnetic_field_values_[2]);

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
