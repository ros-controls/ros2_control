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

#ifndef TEST_MAGNETIC_FIELD_SENSOR_HPP_
#define TEST_MAGNETIC_FIELD_SENSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "semantic_components/magnetic_field_sensor.hpp"

// implementing and friending so we can access member variables
class TestableMagneticFieldSensor : public semantic_components::MagneticFieldSensor
{
  FRIEND_TEST(MagneticFieldSensorTest, validate_all);

public:
  // Use generation of interface names
  explicit TestableMagneticFieldSensor(const std::string & name) : MagneticFieldSensor(name) {}

  virtual ~TestableMagneticFieldSensor() = default;
};

class MagneticFieldSensorTest : public ::testing::Test
{
public:
  void SetUp()
  {
    full_interface_names_.reserve(size_);
    for (auto index = 0u; index < size_; ++index)
    {
      full_interface_names_.emplace_back(
        sensor_name_ + "/" + magnetic_field_interface_names_[index]);
    }
  }

  void TearDown();

protected:
  const size_t size_ = 3;
  const std::string sensor_name_ = "test_magnetometer";
  std::array<double, 3> magnetic_field_values_ = {{4.4, 5.5, 6.6}};
  std::unique_ptr<TestableMagneticFieldSensor> magnetic_field_sensor_;

  std::vector<std::string> full_interface_names_;
  const std::vector<std::string> magnetic_field_interface_names_ = {
    "magnetic_field.x", "magnetic_field.y", "magnetic_field.z"};
};

#endif  // TEST_MAGNETIC_FIELD_SENSOR_HPP_
