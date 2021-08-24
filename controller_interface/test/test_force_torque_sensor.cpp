// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

/*
 * Authors: Subhas Das, Denis Stogl
 */

#include "test_force_torque_sensor.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

void ForceTorqueSensorTest::TearDown() { force_torque_sensor_.reset(nullptr); }

TEST_F(ForceTorqueSensorTest, validate_all_with_default_names)
{
  // create the force torque sensor
  force_torque_sensor_ = std::make_unique<TestableForceTorqueSensor>(sensor_name_);

  // validate the component name
  ASSERT_EQ(force_torque_sensor_->name_, sensor_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(force_torque_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.capacity(), size_);

  // validate the default interface_names_
  ASSERT_TRUE(std::equal(
    force_torque_sensor_->interface_names_.begin(), force_torque_sensor_->interface_names_.end(),
    full_interface_names_.begin(), full_interface_names_.end()));

  // get the interface names
  std::vector<std::string> interface_names = force_torque_sensor_->get_state_interface_names();

  // assign values to force
  hardware_interface::StateInterface force_x{
    sensor_name_, fts_interface_names_[0], &force_values_[0]};
  hardware_interface::StateInterface force_y{
    sensor_name_, fts_interface_names_[1], &force_values_[1]};
  hardware_interface::StateInterface force_z{
    sensor_name_, fts_interface_names_[2], &force_values_[2]};

  // assign values to torque
  hardware_interface::StateInterface torque_x{
    sensor_name_, fts_interface_names_[3], &torque_values_[0]};
  hardware_interface::StateInterface torque_y{
    sensor_name_, fts_interface_names_[4], &torque_values_[1]};
  hardware_interface::StateInterface torque_z{
    sensor_name_, fts_interface_names_[5], &torque_values_[2]};

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(6);

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(torque_y);
  temp_state_interfaces.emplace_back(force_z);
  temp_state_interfaces.emplace_back(force_x);
  temp_state_interfaces.emplace_back(torque_z);
  temp_state_interfaces.emplace_back(torque_x);
  temp_state_interfaces.emplace_back(force_y);

  // now call the function to make them in order like interface_names
  force_torque_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), size_);

  // validate the force values
  std::array<double, 3> temp_force_values = force_torque_sensor_->get_forces();
  ASSERT_EQ(temp_force_values, force_values_);

  // validate the torque values
  std::array<double, 3> temp_torque_values = force_torque_sensor_->get_torques();
  ASSERT_EQ(temp_torque_values, torque_values_);

  // validate get_values_as_message
  geometry_msgs::msg::Wrench temp_message;
  ASSERT_TRUE(force_torque_sensor_->get_values_as_message(temp_message));
  ASSERT_EQ(temp_message.force.x, force_values_[0]);
  ASSERT_EQ(temp_message.force.y, force_values_[1]);
  ASSERT_EQ(temp_message.force.z, force_values_[2]);
  ASSERT_EQ(temp_message.torque.x, torque_values_[0]);
  ASSERT_EQ(temp_message.torque.y, torque_values_[1]);
  ASSERT_EQ(temp_message.torque.z, torque_values_[2]);

  // release the state_interfaces_
  force_torque_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), 0u);
}

TEST_F(ForceTorqueSensorTest, validate_all_with_custom_names)
{
  // create the force torque sensor with force.x, force.z and torque.y, torque.z
  force_torque_sensor_ = std::make_unique<TestableForceTorqueSensor>(
    sensor_name_ + "/" + "force.x", "", sensor_name_ + "/" + "force.z", "",
    sensor_name_ + "/" + "torque.y", sensor_name_ + "/" + "torque.z");

  // validate the component name
  // it should be "" as we specified the interface's names explicitly
  ASSERT_EQ(force_torque_sensor_->name_, "");

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(force_torque_sensor_->interface_names_.size(), 4u);
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.capacity(), size_);

  // validate the default interface_names_
  ASSERT_EQ(force_torque_sensor_->interface_names_[0], full_interface_names_[0]);
  ASSERT_EQ(force_torque_sensor_->interface_names_[1], full_interface_names_[2]);
  ASSERT_EQ(force_torque_sensor_->interface_names_[2], full_interface_names_[4]);
  ASSERT_EQ(force_torque_sensor_->interface_names_[3], full_interface_names_[5]);

  // get the interface names
  std::vector<std::string> interface_names = force_torque_sensor_->get_state_interface_names();

  // assign values to force.x and force.z
  hardware_interface::StateInterface force_x{
    sensor_name_, fts_interface_names_[0], &force_values_[0]};
  hardware_interface::StateInterface force_z{
    sensor_name_, fts_interface_names_[2], &force_values_[2]};

  // assign values to torque.y and torque.z
  hardware_interface::StateInterface torque_y{
    sensor_name_, fts_interface_names_[4], &torque_values_[1]};
  hardware_interface::StateInterface torque_z{
    sensor_name_, fts_interface_names_[5], &torque_values_[2]};

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(4);

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(torque_y);
  temp_state_interfaces.emplace_back(force_z);
  temp_state_interfaces.emplace_back(force_x);
  temp_state_interfaces.emplace_back(torque_z);

  // now call the function to make them in order like interface_names
  force_torque_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), 4u);

  // validate the force values
  std::array<double, 3> temp_force_values = force_torque_sensor_->get_forces();
  ASSERT_EQ(temp_force_values[0], force_values_[0]);
  ASSERT_TRUE(std::isnan(temp_force_values[1]));
  ASSERT_EQ(temp_force_values[2], force_values_[2]);

  // validate the torque values
  std::array<double, 3> temp_torque_values = force_torque_sensor_->get_torques();
  ASSERT_TRUE(std::isnan(temp_torque_values[0]));
  ASSERT_EQ(temp_torque_values[1], torque_values_[1]);
  ASSERT_EQ(temp_torque_values[2], torque_values_[2]);

  // validate get_values_as_message
  geometry_msgs::msg::Wrench temp_message;
  ASSERT_TRUE(force_torque_sensor_->get_values_as_message(temp_message));
  ASSERT_EQ(temp_message.force.x, force_values_[0]);
  ASSERT_TRUE(std::isnan(temp_message.force.y));
  ASSERT_EQ(temp_message.force.z, force_values_[2]);
  ASSERT_TRUE(std::isnan(temp_message.torque.x));
  ASSERT_EQ(temp_message.torque.y, torque_values_[1]);
  ASSERT_EQ(temp_message.torque.z, torque_values_[2]);

  // release the state_interfaces_
  force_torque_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), 0u);
}

TEST_F(ForceTorqueSensorTest, validate_all_custom_names)
{
  // create the force torque sensor with force.x, force.z and torque.y, torque.z
  force_torque_sensor_ = std::make_unique<TestableForceTorqueSensor>(
    sensor_name_ + "/" + "force.x", sensor_name_ + "/" + "force.y", sensor_name_ + "/" + "force.z",
    sensor_name_ + "/" + "torque.x", sensor_name_ + "/" + "torque.y",
    sensor_name_ + "/" + "torque.z");

  // validate the component name
  // it should be "" as we specified the interface's names explicitly
  ASSERT_EQ(force_torque_sensor_->name_, "");

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(force_torque_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.capacity(), size_);

  // validate the custom interface_names_
  ASSERT_EQ(force_torque_sensor_->interface_names_[0], sensor_name_ + "/" + "force.x");
  ASSERT_EQ(force_torque_sensor_->interface_names_[1], sensor_name_ + "/" + "force.y");
  ASSERT_EQ(force_torque_sensor_->interface_names_[2], sensor_name_ + "/" + "force.z");
  ASSERT_EQ(force_torque_sensor_->interface_names_[3], sensor_name_ + "/" + "torque.x");
  ASSERT_EQ(force_torque_sensor_->interface_names_[4], sensor_name_ + "/" + "torque.y");
  ASSERT_EQ(force_torque_sensor_->interface_names_[5], sensor_name_ + "/" + "torque.z");

  // assign values to force
  hardware_interface::StateInterface force_x{
    sensor_name_, fts_interface_names_[0], &force_values_[0]};
  hardware_interface::StateInterface force_y{
    sensor_name_, fts_interface_names_[1], &force_values_[1]};
  hardware_interface::StateInterface force_z{
    sensor_name_, fts_interface_names_[2], &force_values_[2]};

  // assign values to torque
  hardware_interface::StateInterface torque_x{
    sensor_name_, fts_interface_names_[3], &torque_values_[0]};
  hardware_interface::StateInterface torque_y{
    sensor_name_, fts_interface_names_[4], &torque_values_[1]};
  hardware_interface::StateInterface torque_z{
    sensor_name_, fts_interface_names_[5], &torque_values_[2]};

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(torque_y);
  temp_state_interfaces.emplace_back(force_z);
  temp_state_interfaces.emplace_back(force_x);
  temp_state_interfaces.emplace_back(torque_z);
  temp_state_interfaces.emplace_back(torque_x);
  temp_state_interfaces.emplace_back(force_y);

  // now call the function to make them in order like interface_names
  force_torque_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), size_);

  // validate the force values
  std::array<double, 3> temp_force_values = force_torque_sensor_->get_forces();
  ASSERT_EQ(temp_force_values[0], force_values_[0]);
  ASSERT_EQ(temp_force_values[1], force_values_[1]);
  ASSERT_EQ(temp_force_values[2], force_values_[2]);

  // validate the torque values
  std::array<double, 3> temp_torque_values = force_torque_sensor_->get_torques();
  ASSERT_EQ(temp_torque_values[0], torque_values_[0]);
  ASSERT_EQ(temp_torque_values[1], torque_values_[1]);
  ASSERT_EQ(temp_torque_values[2], torque_values_[2]);

  // validate get_values_as_message
  geometry_msgs::msg::Wrench temp_message;
  ASSERT_TRUE(force_torque_sensor_->get_values_as_message(temp_message));
  ASSERT_EQ(temp_message.force.x, force_values_[0]);
  ASSERT_EQ(temp_message.force.y, force_values_[1]);
  ASSERT_EQ(temp_message.force.z, force_values_[2]);
  ASSERT_EQ(temp_message.torque.x, torque_values_[0]);
  ASSERT_EQ(temp_message.torque.y, torque_values_[1]);
  ASSERT_EQ(temp_message.torque.z, torque_values_[2]);

  // release the state_interfaces_
  force_torque_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(force_torque_sensor_->state_interfaces_.size(), 0u);
}
