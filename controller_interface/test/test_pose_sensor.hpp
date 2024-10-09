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

#ifndef TEST_POSE_SENSOR_HPP_
#define TEST_POSE_SENSOR_HPP_

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "semantic_components/pose_sensor.hpp"

class TestablePoseSensor : public semantic_components::PoseSensor
{
  FRIEND_TEST(PoseSensorTest, validate_all);

public:
  // Use default interface names
  explicit TestablePoseSensor(const std::string & name) : PoseSensor{name} {}

  virtual ~TestablePoseSensor() = default;
};

class PoseSensorTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  const size_t size_ = 7;
  const std::string sensor_name_ = "test_pose_sensor";

  std::vector<std::string> full_interface_names_;
  const std::vector<std::string> interface_names_ = {
    "position.x",    "position.y",    "position.z",   "orientation.x",
    "orientation.y", "orientation.z", "orientation.w"};

  std::array<double, 3> position_values_ = {{1.1, 2.2, 3.3}};
  std::array<double, 4> orientation_values_ = {{4.4, 5.5, 6.6, 7.7}};

  std::unique_ptr<TestablePoseSensor> pose_sensor_;
};

#endif  // TEST_POSE_SENSOR_HPP_
