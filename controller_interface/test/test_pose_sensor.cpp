// Copyright (c) 2024, FZI Forschungszentrum Informatik
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
#include "test_pose_sensor.hpp"

void PoseSensorTest::SetUp()
{
  full_interface_names_.reserve(size_);
  for (const auto & interface_name : interface_names_)
  {
    full_interface_names_.emplace_back(sensor_name_ + '/' + interface_name);
  }
}

void PoseSensorTest::TearDown() { pose_sensor_.reset(nullptr); }

TEST_F(PoseSensorTest, validate_all)
{
  // Create sensor
  pose_sensor_ = std::make_unique<TestablePoseSensor>(sensor_name_);
  EXPECT_EQ(pose_sensor_->name_, sensor_name_);

  // Validate reserved space for interface_names_ and state_interfaces_
  // As state_interfaces_ are not defined yet, use capacity()
  ASSERT_EQ(pose_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(pose_sensor_->state_interfaces_.capacity(), size_);

  // Validate default interface_names_
  EXPECT_TRUE(std::equal(
    pose_sensor_->interface_names_.cbegin(), pose_sensor_->interface_names_.cend(),
    full_interface_names_.cbegin(), full_interface_names_.cend()));

  // Get interface names
  std::vector<std::string> interface_names = pose_sensor_->get_state_interface_names();

  // Assign values to position
  hardware_interface::StateInterface position_x{
    sensor_name_, interface_names_[0], &position_values_[0]};
  hardware_interface::StateInterface position_y{
    sensor_name_, interface_names_[1], &position_values_[1]};
  hardware_interface::StateInterface position_z{
    sensor_name_, interface_names_[2], &position_values_[2]};

  // Assign values to orientation
  hardware_interface::StateInterface orientation_x{
    sensor_name_, interface_names_[3], &orientation_values_[0]};
  hardware_interface::StateInterface orientation_y{
    sensor_name_, interface_names_[4], &orientation_values_[1]};
  hardware_interface::StateInterface orientation_z{
    sensor_name_, interface_names_[5], &orientation_values_[2]};
  hardware_interface::StateInterface orientation_w{
    sensor_name_, interface_names_[6], &orientation_values_[3]};

  // Create state interface vector in jumbled order
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(7);

  temp_state_interfaces.emplace_back(position_z);
  temp_state_interfaces.emplace_back(orientation_y);
  temp_state_interfaces.emplace_back(orientation_x);
  temp_state_interfaces.emplace_back(position_x);
  temp_state_interfaces.emplace_back(orientation_w);
  temp_state_interfaces.emplace_back(position_y);
  temp_state_interfaces.emplace_back(orientation_z);

  // Assign interfaces
  pose_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);
  EXPECT_EQ(pose_sensor_->state_interfaces_.size(), size_);

  // Validate correct position and orientation
  EXPECT_EQ(pose_sensor_->get_position(), position_values_);
  EXPECT_EQ(pose_sensor_->get_orientation(), orientation_values_);

  // Validate generated message
  geometry_msgs::msg::Pose temp_message;
  ASSERT_TRUE(pose_sensor_->get_values_as_message(temp_message));
  EXPECT_EQ(temp_message.position.x, position_values_[0]);
  EXPECT_EQ(temp_message.position.y, position_values_[1]);
  EXPECT_EQ(temp_message.position.z, position_values_[2]);
  EXPECT_EQ(temp_message.orientation.x, orientation_values_[0]);
  EXPECT_EQ(temp_message.orientation.y, orientation_values_[1]);
  EXPECT_EQ(temp_message.orientation.z, orientation_values_[2]);
  EXPECT_EQ(temp_message.orientation.w, orientation_values_[3]);

  // Release state interfaces
  pose_sensor_->release_interfaces();
  ASSERT_EQ(pose_sensor_->state_interfaces_.size(), 0);
}
