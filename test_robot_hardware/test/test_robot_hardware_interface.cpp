// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "test_robot_hardware/test_robot_hardware.hpp"

using hw_ret = hardware_interface::return_type;

class TestRobotHardwareInterface : public ::testing::Test
{
protected:
  void SetUp()
  {
    joint_names_ = robot_.joint_names;

    joint_pos_values_ = robot_.pos_dflt_values;
    joint_vel_values_ = robot_.vel_dflt_values;
    joint_eff_values_ = robot_.eff_dflt_values;

    joint_pos_cmd_values_ = robot_.pos_dflt_values;
    joint_vel_cmd_values_ = robot_.vel_dflt_values;
    joint_eff_cmd_values_ = robot_.eff_dflt_values;
  }

  test_robot_hardware::TestRobotHardware robot_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_pos_values_;
  std::vector<double> joint_vel_values_;
  std::vector<double> joint_eff_values_;

  std::vector<double> joint_pos_cmd_values_;
  std::vector<double> joint_vel_cmd_values_;
  std::vector<double> joint_eff_cmd_values_;
};

TEST_F(TestRobotHardwareInterface, initialize) {
  EXPECT_EQ(hw_ret::OK, robot_.init());
}

TEST_F(TestRobotHardwareInterface, get_registered_joint_handles) {
  robot_.init();

  auto ret = hw_ret::ERROR;
  for (auto i = 0u; i < 3u; ++i) {
    auto get_handle = [&](const std::string joint_name, const std::string interface_name)
      {
        hardware_interface::JointHandle joint_handle(joint_name, interface_name);
        ret = robot_.get_joint_handle(joint_handle);
        return joint_handle;
      };

    EXPECT_EQ(get_handle(joint_names_[i], "position").get_value(), joint_pos_values_[i]);
    EXPECT_EQ(get_handle(joint_names_[i], "velocity").get_value(), joint_vel_values_[i]);
    EXPECT_EQ(get_handle(joint_names_[i], "effort").get_value(), joint_eff_values_[i]);

    EXPECT_EQ(get_handle(joint_names_[i], "position_command").get_value(), joint_pos_values_[i]);
    EXPECT_EQ(get_handle(joint_names_[i], "velocity_command").get_value(), joint_vel_values_[i]);
    EXPECT_EQ(get_handle(joint_names_[i], "effort_command").get_value(), joint_eff_values_[i]);
  }

  // 3 joints * 6 interfaces
  EXPECT_EQ(robot_.get_registered_joints().size(), 3u * 6u);
}
