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

#include "test_led_rgb_device.hpp"

void LedDeviceTest::SetUp()
{
  full_cmd_interface_names_.reserve(size_);
  for (const auto & interface_name : interface_names_)
  {
    full_cmd_interface_names_.emplace_back(device_name_ + '/' + interface_name);
  }
}

void LedDeviceTest::TearDown() { led_device_.reset(nullptr); }

TEST_F(LedDeviceTest, validate_all)
{
  // Create device
  led_device_ = std::make_unique<TestableLedDevice>(
    device_name_, interface_names_[0], interface_names_[1], interface_names_[2]);
  EXPECT_EQ(led_device_->name_, device_name_);

  // Validate reserved space for interface_names_ and command_interfaces_
  // As command_interfaces_ are not defined yet, use capacity()
  ASSERT_EQ(led_device_->interface_names_.size(), size_);
  ASSERT_EQ(led_device_->command_interfaces_.capacity(), size_);

  // Validate default interface_names_
  EXPECT_TRUE(std::equal(
    led_device_->interface_names_.cbegin(), led_device_->interface_names_.cend(),
    full_cmd_interface_names_.cbegin(), full_cmd_interface_names_.cend()));

  // Get interface names
  std::vector<std::string> interface_names = led_device_->get_command_interface_names();

  // Assign values to position
  hardware_interface::CommandInterface led_r{device_name_, interface_names_[0], &led_values_[0]};
  hardware_interface::CommandInterface led_g{device_name_, interface_names_[1], &led_values_[1]};
  hardware_interface::CommandInterface led_b{device_name_, interface_names_[2], &led_values_[2]};

  // Create command interface vector in jumbled order
  std::vector<hardware_interface::LoanedCommandInterface> temp_command_interfaces;
  temp_command_interfaces.reserve(3);
  temp_command_interfaces.emplace_back(led_r);
  temp_command_interfaces.emplace_back(led_g);
  temp_command_interfaces.emplace_back(led_b);

  // Assign interfaces
  led_device_->assign_loaned_command_interfaces(temp_command_interfaces);
  EXPECT_EQ(led_device_->command_interfaces_.size(), size_);

  // Validate correct assignment
  const std::vector<double> test_led_values_cmd = {0.1, 0.2, 0.3};
  EXPECT_TRUE(led_device_->set_values(test_led_values_cmd));

  EXPECT_EQ(led_values_[0], test_led_values_cmd[0]);
  EXPECT_EQ(led_values_[1], test_led_values_cmd[1]);
  EXPECT_EQ(led_values_[2], test_led_values_cmd[2]);

  // Validate correct assignment from message
  std_msgs::msg::ColorRGBA temp_message;
  temp_message.r = static_cast<float>(test_led_values_cmd[0]);
  temp_message.g = static_cast<float>(test_led_values_cmd[1]);
  temp_message.b = static_cast<float>(test_led_values_cmd[2]);
  EXPECT_TRUE(led_device_->set_values_from_message(temp_message));

  double float_tolerance = 1e-6;
  EXPECT_NEAR(led_values_[0], test_led_values_cmd[0], float_tolerance);
  EXPECT_NEAR(led_values_[1], test_led_values_cmd[1], float_tolerance);
  EXPECT_NEAR(led_values_[2], test_led_values_cmd[2], float_tolerance);

  // Release command interfaces
  led_device_->release_interfaces();
  ASSERT_EQ(led_device_->command_interfaces_.size(), 0);
}
