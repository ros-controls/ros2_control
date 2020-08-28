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
#include <vector>

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include "test_robot_hardware/visibility_control.h"

namespace test_robot_hardware
{
// TODO(karsten1987): Maybe visibility macros on class level
// as all members are publically exposed
class TestRobotHardware : public hardware_interface::RobotHardware
{
public:
  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type
  init();

  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type
  read();

  TEST_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type
  write();

  std::string read_op_handle_name1 = "read1";
  std::string read_op_handle_name2 = "read2";
  std::string write_op_handle_name1 = "write1";
  std::string write_op_handle_name2 = "write2";

  std::vector<std::string> actuator_names = {"actuator1", "actuator2", "actuator3"};
  std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};

  std::vector<double> pos_dflt_values = {1.1, 2.1, 3.1};
  std::vector<double> vel_dflt_values = {1.2, 2.2, 3.2};
  std::vector<double> eff_dflt_values = {1.3, 2.3, 3.3};

  bool read1 = false;
  bool read2 = false;
  bool write1 = false;
  bool write2 = false;

  hardware_interface::OperationModeHandle read_op_handle1;
  hardware_interface::OperationModeHandle read_op_handle2;

  hardware_interface::OperationModeHandle write_op_handle1;
  hardware_interface::OperationModeHandle write_op_handle2;

  const rclcpp::Logger logger = rclcpp::get_logger("test_robot_hardware");
};

}  // namespace test_robot_hardware
#endif  // TEST_ROBOT_HARDWARE__TEST_ROBOT_HARDWARE_HPP_
