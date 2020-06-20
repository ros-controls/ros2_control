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

using namespace hardware_interface;

namespace
{
class MyTestRobotHardware : public RobotHardware
{
  public:
    hardware_interface_ret_t init() override
    {
        return HW_RET_OK;
    }

    hardware_interface_ret_t read() override
    {
        return HW_RET_OK;
    }

    hardware_interface_ret_t write() override
    {
        return HW_RET_OK;
    }
};
}

namespace testing
{
class TestRobotHardwareInterface : public Test
{
  protected:
    void SetUp()
    {
        robot.init();
    }

    MyTestRobotHardware robot;
};

TEST_F(TestRobotHardwareInterface, initialize)
{
    MyTestRobotHardware local_robot;
    EXPECT_EQ(HW_RET_OK, local_robot.init());
}

TEST_F(TestRobotHardwareInterface, can_not_register_broken_handles)
{
    JointCommandHandle broken_command_handle;
    EXPECT_EQ(HW_RET_ERROR, robot.register_joint_command_handle(&broken_command_handle));
    JointStateHandle broken_state_handle;
    EXPECT_EQ(HW_RET_ERROR, robot.register_joint_state_handle(&broken_state_handle));
    OperationModeHandle broken_op_mode_handle;
    EXPECT_EQ(HW_RET_ERROR, robot.register_operation_mode_handle(&broken_op_mode_handle));
}

}  // namespace testing