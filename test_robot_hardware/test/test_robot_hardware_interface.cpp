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

class TestRobotHardwareInterface : public ::testing::Test
{
protected:
  void SetUp()
  {
    joint_names = {robot.joint_name1, robot.joint_name2, robot.joint_name3};
    joint_pos_values = {robot.pos1, robot.pos2, robot.pos3};
    joint_vel_values = {robot.vel1, robot.vel2, robot.vel3};
    joint_eff_values = {robot.eff1, robot.eff2, robot.eff3};
    joint_cmd_values = {robot.cmd1, robot.cmd2, robot.cmd3};
  }

  test_robot_hardware::TestRobotHardware robot;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_values;
  std::vector<double> joint_vel_values;
  std::vector<double> joint_eff_values;
  std::vector<double> joint_cmd_values;
};

TEST_F(TestRobotHardwareInterface, initialize) {
  EXPECT_EQ(hardware_interface::HW_RET_OK, robot.init());
}

TEST_F(TestRobotHardwareInterface, get_registered_joint_handles) {
  robot.init();

  auto ret = hardware_interface::HW_RET_ERROR;
  for (auto i = 0u; i < 3u; ++i) {
    const hardware_interface::JointStateHandle * js_ptr = nullptr;
    ret = robot.get_joint_state_handle(joint_names[i], &js_ptr);
    EXPECT_EQ(hardware_interface::HW_RET_OK, ret);
    EXPECT_EQ(joint_pos_values[i], js_ptr->get_position());
    EXPECT_EQ(joint_vel_values[i], js_ptr->get_velocity());
    EXPECT_EQ(joint_eff_values[i], js_ptr->get_effort());
    ret = hardware_interface::HW_RET_ERROR;
  }

  auto registered_joint_handles = robot.get_registered_joint_state_handles();
  EXPECT_EQ(3u, registered_joint_handles.size());
}

TEST_F(TestRobotHardwareInterface, get_registered_command_handles) {
  robot.init();

  auto ret = hardware_interface::HW_RET_ERROR;
  for (auto i = 0u; i < 3u; ++i) {
    hardware_interface::JointCommandHandle * jcmd_ptr = nullptr;
    ret = robot.get_joint_command_handle(joint_names[i], &jcmd_ptr);
    EXPECT_EQ(hardware_interface::HW_RET_OK, ret);
    EXPECT_EQ(joint_cmd_values[i], jcmd_ptr->get_cmd());
    ret = hardware_interface::HW_RET_ERROR;
  }

  auto registered_command_handles = robot.get_registered_joint_command_handles();
  EXPECT_EQ(3u, registered_command_handles.size());
}
