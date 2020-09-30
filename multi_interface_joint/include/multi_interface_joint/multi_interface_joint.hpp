// Copyright 2020 ros2_control Development Team
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

#ifndef MULTI_INTERFACE_JOINT__MULTI_INTERFACE_JOINT_HPP_
#define MULTI_INTERFACE_JOINT__MULTI_INTERFACE_JOINT_HPP_

#include <string>
#include <vector>

#include "multi_interface_joint/visibility_control.h"

#include "hardware_interface/components/joint_interface.hpp"

namespace multi_interface_joint
{

class MultiInterfaceJoint : public hardware_interface::components::JointInterface
{
public:
  MultiInterfaceJoint() = default;

  virtual ~MultiInterfaceJoint() = default;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type configure(
    const hardware_interface::components::ComponentInfo & joint_info) override;

  MULTI_INTERFACE_JOINT_PUBLIC
  std::vector<std::string> get_command_interfaces() const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  std::vector<std::string> get_state_interfaces() const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type get_command(
    std::vector<double> & command,
    const std::vector<std::string> & interfaces) const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type get_command(
    std::vector<double> & command) const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type set_command(
    const std::vector<double> & command,
    const std::vector<std::string> & interfaces) override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type set_command(const std::vector<double> & command) override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type get_state(
    std::vector<double> & state,
    const std::vector<std::string> & interfaces) const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type get_state(std::vector<double> & state) const override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type set_state(
    const std::vector<double> & state,
    const std::vector<std::string> & interfaces) override;

  MULTI_INTERFACE_JOINT_PUBLIC
  hardware_interface::return_type set_state(const std::vector<double> & state) override;

protected:
  std::vector<std::string> command_interfaces_;
  std::vector<double> command_values_;

  std::vector<std::string> state_interfaces_;
  std::vector<double> state_values_;
};

}  // namespace multi_interface_joint

#endif  // MULTI_INTERFACE_JOINT__MULTI_INTERFACE_JOINT_HPP_
