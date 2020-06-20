// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "gmock/gmock.h"

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace hw = hardware_interface;

namespace
{
class MyTestRobotHardware : public hw::RobotHardware
{
public:
  hw::hardware_interface_ret_t init() override
  {
    return hw::HW_RET_OK;
  }

  hw::hardware_interface_ret_t read() override
  {
    return hw::HW_RET_OK;
  }

  hw::hardware_interface_ret_t write() override
  {
    return hw::HW_RET_OK;
  }
};

const auto JOINT_NAME = "joint_1";
const auto NEW_JOINT_NAME = "joint_2";
}  // namespace

namespace testing
{
class TestRobotHardwareInterface : public Test
{
protected:
  void SetUp()
  {
    robot.init();
  }

  void SetUpHandles()
  {
    robot.register_joint_command_handle(&pos_command_handle);
    robot.register_joint_state_handle(&state_handle);
    robot.register_operation_mode_handle(&op_mode_handle);
  }

  void SetUpNewHandles()
  {
    robot.register_joint_command_handle(&new_pos_command_handle);
    robot.register_joint_state_handle(&new_state_handle);
    robot.register_operation_mode_handle(&new_op_mode_handle);
  }

  MyTestRobotHardware robot;
  double pos_command_value = 0.0;
  hw::JointCommandHandle pos_command_handle{JOINT_NAME, &pos_command_value};
  double velocity_state_value = 0.0;
  double effort_state_value = 0.0;
  hw::JointStateHandle state_handle{JOINT_NAME, &pos_command_value, &velocity_state_value,
    &effort_state_value};
  hw::OperationMode mode = hw::OperationMode::ACTIVE;
  hw::OperationModeHandle op_mode_handle{JOINT_NAME, &mode};

  hw::JointCommandHandle new_pos_command_handle{NEW_JOINT_NAME, &pos_command_value};
  hw::JointStateHandle new_state_handle{NEW_JOINT_NAME, &pos_command_value, &velocity_state_value,
    &effort_state_value};
  hw::OperationModeHandle new_op_mode_handle{NEW_JOINT_NAME, &mode};
};

TEST_F(TestRobotHardwareInterface, can_not_register_broken_handles)
{
  hw::JointCommandHandle broken_command_handle;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_joint_command_handle(&broken_command_handle));
  hw::JointStateHandle broken_state_handle;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_joint_state_handle(&broken_state_handle));
  hw::OperationModeHandle broken_op_mode_handle;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_operation_mode_handle(&broken_op_mode_handle));
}

TEST_F(TestRobotHardwareInterface, can_register_proper_handles)
{
  EXPECT_EQ(hw::HW_RET_OK, robot.register_joint_command_handle(&pos_command_handle));
  EXPECT_EQ(hw::HW_RET_OK, robot.register_joint_state_handle(&state_handle));
  EXPECT_EQ(hw::HW_RET_OK, robot.register_operation_mode_handle(&op_mode_handle));
}

TEST_F(TestRobotHardwareInterface, cannot_double_register_handles)
{
  SetUpHandles();

  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_joint_command_handle(&pos_command_handle));
  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_joint_state_handle(&state_handle));
  EXPECT_EQ(hw::HW_RET_ERROR, robot.register_operation_mode_handle(&op_mode_handle));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_joint_names)
{
  SetUpHandles();
  EXPECT_THAT(robot.get_registered_joint_names(), ElementsAre(JOINT_NAME));

  SetUpNewHandles();
  EXPECT_THAT(robot.get_registered_joint_names(), UnorderedElementsAre(JOINT_NAME, NEW_JOINT_NAME));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_state_handles)
{
  SetUpHandles();
  EXPECT_THAT(robot.get_registered_joint_state_handles(), SizeIs(1));

  SetUpNewHandles();
  EXPECT_THAT(robot.get_registered_joint_state_handles(), SizeIs(2));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_command_handles)
{
  SetUpHandles();
  EXPECT_THAT(robot.get_registered_joint_command_handles(), SizeIs(1));

  SetUpNewHandles();
  EXPECT_THAT(robot.get_registered_joint_command_handles(), SizeIs(2));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_op_mode_handles)
{
  SetUpHandles();
  EXPECT_THAT(robot.get_registered_operation_mode_handles(), SizeIs(1));

  SetUpNewHandles();
  EXPECT_THAT(robot.get_registered_operation_mode_handles(), SizeIs(2));
}

TEST_F(TestRobotHardwareInterface, can_get_handles_by_name)
{
  SetUpHandles();

  const hw::JointStateHandle * state_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_joint_state_handle(JOINT_NAME, &state_handle));
  state_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.get_joint_state_handle(NEW_JOINT_NAME, &state_handle));

  hw::JointCommandHandle * cmd_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_joint_command_handle(JOINT_NAME, &cmd_handle));
  cmd_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.get_joint_command_handle(NEW_JOINT_NAME, &cmd_handle));

  hw::OperationModeHandle * op_mode_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_operation_mode_handle(JOINT_NAME, &op_mode_handle));
  op_mode_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_ERROR, robot.get_operation_mode_handle(NEW_JOINT_NAME, &op_mode_handle));

  SetUpNewHandles();
  state_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_joint_state_handle(NEW_JOINT_NAME, &state_handle));
  cmd_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_joint_command_handle(NEW_JOINT_NAME, &cmd_handle));
  op_mode_handle = nullptr;
  EXPECT_EQ(hw::HW_RET_OK, robot.get_operation_mode_handle(NEW_JOINT_NAME, &op_mode_handle));
}

}  // namespace testing
