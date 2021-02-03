// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Jafar Abdi, Denis Stogl

#ifndef FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_
#define FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace fake_components
{

class GenericSystem : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  return_type
  configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  return_type start() override
  {
    status_ = hardware_interface::status::STARTED;
    return return_type::OK;
  }

  return_type stop() override
  {
    status_ = hardware_interface::status::STOPPED;
    return return_type::OK;
  }

  return_type read() override;

  return_type write() override
  {
    return return_type::OK;
  }

private:
  // add variables for "standardized" interfaces in the following order
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT
  };
  /// The size of this vector is (standard_interfaces_ x nr_joints)
  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
};

typedef GenericSystem GenericRobot;

}  // namespace fake_components

#endif  // FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_
