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

#ifndef TEST_FORCE_TORQUE_SENSOR_HPP_
#define TEST_FORCE_TORQUE_SENSOR_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "semantic_components/force_torque_sensor.hpp"

// implementing and friending so we can access member varibles
class TestableForceTorqueSensor : public semantic_components::
  ForceTorqueSensor
{
  FRIEND_TEST(ForceTorqueSensorTest, validate_all_with_default_names);
  FRIEND_TEST(ForceTorqueSensorTest, validate_all_with_custom_names);
  FRIEND_TEST(ForceTorqueSensorTest, validate_all_custom_names);

public:
  // Use generation of interface names
  explicit TestableForceTorqueSensor(const std::string & name)
  : ForceTorqueSensor(name)
  {}
  // Use custom interface names
  explicit TestableForceTorqueSensor(
    const std::string & interface_force_x, const std::string & interface_force_y,
    const std::string & interface_force_z, const std::string & interface_torque_x,
    const std::string & interface_torque_y, const std::string & interface_torque_z)
  : ForceTorqueSensor(
      interface_force_x, interface_force_y, interface_force_z,
      interface_torque_x, interface_torque_y, interface_torque_z)
  {}
};

class ForceTorqueSensorTest : public ::testing::Test
{
public:
  void TearDown();

protected:
  const size_t size_ = 6;
  const std::string sensor_name_ = "test_FTS";
  std::array<double, 3> force_values_ = {1.1, 2.2, 3.3};
  std::array<double, 3> torque_values_ = {4.4, 5.5, 6.6};
  std::unique_ptr<TestableForceTorqueSensor> force_torque_sensor_;
};

#endif  // TEST_FORCE_TORQUE_SENSOR_HPP_
