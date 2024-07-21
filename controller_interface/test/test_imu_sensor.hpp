// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#ifndef TEST_IMU_SENSOR_HPP_
#define TEST_IMU_SENSOR_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "semantic_components/imu_sensor.hpp"

// implementing and friending so we can access member variables
class TestableIMUSensor : public semantic_components::IMUSensor
{
  FRIEND_TEST(IMUSensorTest, validate_all);

public:
  // Use generation of interface names
  explicit TestableIMUSensor(const std::string & name) : IMUSensor(name) {}

  virtual ~TestableIMUSensor() = default;
};

class IMUSensorTest : public ::testing::Test
{
public:
  void SetUp()
  {
    full_interface_names_.reserve(size_);
    for (auto index = 0u; index < size_; ++index)
    {
      full_interface_names_.emplace_back(sensor_name_ + "/" + imu_interface_names_[index]);
    }
  }

  void TearDown();

protected:
  const size_t size_ = 10;
  const std::string sensor_name_ = "test_IMU";
  std::array<double, 4> orientation_values_ = {{1.1, 2.2, 3.3, 4.4}};
  std::array<double, 3> angular_velocity_values_ = {{4.4, 5.5, 6.6}};
  std::array<double, 3> linear_acceleration_values_ = {{4.4, 5.5, 6.6}};
  std::unique_ptr<TestableIMUSensor> imu_sensor_;

  std::vector<std::string> full_interface_names_;
  const std::vector<std::string> imu_interface_names_ = {
    "orientation.x",         "orientation.y",        "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",   "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z"};
};

#endif  // TEST_IMU_SENSOR_HPP_
