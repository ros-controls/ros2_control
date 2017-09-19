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

#ifndef TEST_ROBOT_HARDWARE__TEST_ROBOT_HARDWARE_HPP_
#define TEST_ROBOT_HARDWARE__TEST_ROBOT_HARDWARE_HPP_

#include <string>

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "test_robot_hardware/visibility_control.h"

namespace test_robot_hardware
{
// TODO(karsten1987): Maybe visibility macros on class level
// as all members are publically exposed
class TestRobotHardware : public hardware_interface::RobotHardware
{
public:
  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t
  init();

  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t
  read();

  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t
  write();

  std::string joint_name1 = "joint1";
  std::string joint_name2 = "joint2";
  std::string joint_name3 = "joint3";

  std::string read_op_handle_name1 = "read1";
  std::string read_op_handle_name2 = "read2";
  std::string write_op_handle_name1 = "write1";
  std::string write_op_handle_name2 = "write2";

  double pos1 = 1.1;
  double pos2 = 2.2;
  double pos3 = 3.3;

  double vel1 = 1.1;
  double vel2 = 2.2;
  double vel3 = 3.3;

  double eff1 = 1.1;
  double eff2 = 2.2;
  double eff3 = 3.3;

  double cmd1 = 1.1;
  double cmd2 = 2.2;
  double cmd3 = 3.3;

  bool read1 = false;
  bool read2 = false;
  bool write1 = false;
  bool write2 = false;

  hardware_interface::JointStateHandle js1;
  hardware_interface::JointStateHandle js2;
  hardware_interface::JointStateHandle js3;

  hardware_interface::JointCommandHandle jcmd1;
  hardware_interface::JointCommandHandle jcmd2;
  hardware_interface::JointCommandHandle jcmd3;

  hardware_interface::OperationModeHandle read_op_handle1;
  hardware_interface::OperationModeHandle read_op_handle2;

  hardware_interface::OperationModeHandle write_op_handle1;
  hardware_interface::OperationModeHandle write_op_handle2;
};

}  // namespace test_robot_hardware
#endif  // TEST_ROBOT_HARDWARE__TEST_ROBOT_HARDWARE_HPP_
