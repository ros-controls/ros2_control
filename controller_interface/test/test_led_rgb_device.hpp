// Copyright (c) 2024, Sherpa Mobile Robotics
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

#ifndef TEST_LED_RGB_DEVICE_HPP_
#define TEST_LED_RGB_DEVICE_HPP_

#include <gmock/gmock.h>

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "semantic_components/led_rgb_device.hpp"

class TestableLedDevice : public semantic_components::LedRgbDevice
{
  FRIEND_TEST(LedDeviceTest, validate_all);

public:
  TestableLedDevice(
    const std::string & name, const std::string & interface_r, const std::string & interface_g,
    const std::string & interface_b)
  : LedRgbDevice{name, interface_r, interface_g, interface_b}
  {
  }

  virtual ~TestableLedDevice() = default;
};

class LedDeviceTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  const size_t size_ = 3;
  const std::string device_name_ = "test_led_device";

  std::vector<std::string> full_cmd_interface_names_;
  const std::vector<std::string> interface_names_ = {"r", "g", "b"};

  std::array<double, 3> led_values_ = {
    {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
     std::numeric_limits<double>::quiet_NaN()}};

  std::unique_ptr<TestableLedDevice> led_device_;
};

#endif  // TEST_LED_RGB_DEVICE_HPP_
