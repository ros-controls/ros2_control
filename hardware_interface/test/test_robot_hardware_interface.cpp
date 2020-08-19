// Copyright 2020 ros2_control development team
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
using testing::SizeIs;
using testing::Each;
using testing::NotNull;
using testing::UnorderedElementsAre;
using testing::ElementsAre;

namespace
{
class MyTestRobotHardware : public hw::RobotHardware
{
public:
  hw::return_type init() override
  {
    return hw::return_type::OK;
  }

  hw::return_type read() override
  {
    return hw::return_type::OK;
  }

  hw::return_type write() override
  {
    return hw::return_type::OK;
  }
};

constexpr auto JOINT_NAME = "joint_1";
constexpr auto NEW_JOINT_NAME = "joint_2";
}  // namespace

class TestRobotHardwareInterface : public testing::Test
{
protected:
  void SetUp()
  {
    robot_.init();
  }

  void SetUpHandles()
  {
    robot_.register_operation_mode_handle(&op_mode_handle_);
  }

  void SetUpNewHandles()
  {
    robot_.register_operation_mode_handle(&new_op_mode_handle_);
  }

  MyTestRobotHardware robot_;

  hw::OperationMode mode_ = hw::OperationMode::ACTIVE;

  hw::OperationModeHandle op_mode_handle_{JOINT_NAME, &mode_};
  hw::OperationModeHandle new_op_mode_handle_{NEW_JOINT_NAME, &mode_};
};

TEST_F(TestRobotHardwareInterface, can_not_register_broken_handles)
{
  hw::OperationModeHandle broken_op_mode_handle;
  EXPECT_EQ(hw::return_type::ERROR, robot_.register_operation_mode_handle(&broken_op_mode_handle));
}

TEST_F(TestRobotHardwareInterface, can_register_proper_handles)
{
  EXPECT_EQ(hw::return_type::OK, robot_.register_operation_mode_handle(&op_mode_handle_));
}

TEST_F(TestRobotHardwareInterface, cannot_double_register_handles)
{
  SetUpHandles();

  EXPECT_EQ(hw::return_type::ERROR, robot_.register_operation_mode_handle(&op_mode_handle_));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_joint_names)
{
  SetUpHandles();
  // EXPECT_THAT(robot_.get_registered_joint_names(), ElementsAre(JOINT_NAME));

  SetUpNewHandles();
  // EXPECT_THAT(
  //   robot_.get_registered_joint_names(),
  //   UnorderedElementsAre(JOINT_NAME, NEW_JOINT_NAME));
}

TEST_F(TestRobotHardwareInterface, returned_joint_handles_are_not_null)
{
  SetUpHandles();
  SetUpNewHandles();

  const auto op_mode_handles = robot_.get_registered_operation_mode_handles();
  EXPECT_THAT(op_mode_handles, SizeIs(2));
  EXPECT_THAT(op_mode_handles, Each(NotNull()));
}

TEST_F(TestRobotHardwareInterface, can_get_registered_op_mode_handles)
{
  SetUpHandles();
  EXPECT_THAT(robot_.get_registered_operation_mode_handles(), SizeIs(1));

  SetUpNewHandles();
  EXPECT_THAT(robot_.get_registered_operation_mode_handles(), SizeIs(2));
}

TEST_F(TestRobotHardwareInterface, can_get_handles_by_name)
{
  SetUpHandles();

  hw::OperationModeHandle * op_mode_handle = nullptr;
  EXPECT_EQ(hw::return_type::OK, robot_.get_operation_mode_handle(JOINT_NAME, &op_mode_handle));
  op_mode_handle = nullptr;
  EXPECT_EQ(
    hw::return_type::ERROR,
    robot_.get_operation_mode_handle(NEW_JOINT_NAME, &op_mode_handle));

  SetUpNewHandles();
  op_mode_handle = nullptr;
  EXPECT_EQ(hw::return_type::OK, robot_.get_operation_mode_handle(NEW_JOINT_NAME, &op_mode_handle));
}
