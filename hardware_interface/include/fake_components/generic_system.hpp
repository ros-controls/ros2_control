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

#include <chrono>
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
enum StoppingInterface
{
  NONE,
  STOP_POSITION,
  STOP_VELOCITY
};

class HARDWARE_INTERFACE_PUBLIC GenericSystem
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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

  return_type write() override { return return_type::OK; }

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

protected:
  /// Use standard interfaces for joints because they are relevant for dynamic behaviour
  /**
   * By splitting the standard interfaces from other type, the users are able to inherit this
   * class and simply create small "simulation" with desired dynamic behaviour.
   * The advantage over using Gazebo is that enables "quick & dirty" tests of robot's URDF and
   * controllers.
   */
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  const size_t POSITION_INTERFACE_INDEX = 0;
  const size_t VELOCITY_INTERFACE_INDEX = 1;

  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;
  double joint_vel_commands_[2];

  std::vector<std::string> other_interfaces_;
  /// The size of this vector is (other_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> other_commands_;
  std::vector<std::vector<double>> other_states_;

  std::vector<std::string> sensor_interfaces_;
  /// The size of this vector is (other_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> sensor_fake_commands_;
  std::vector<std::vector<double>> sensor_states_;

private:
  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces);

  bool fake_sensor_command_interfaces_;
  double position_state_following_offset_;
  std::string custom_interface_with_following_offset_;
  size_t index_custom_interface_with_following_offset_;
  bool command_propagation_disabled_;

  // resources switching aux vars
  std::vector<uint> stop_modes_;
  std::vector<std::string> start_modes_;
  bool position_controller_running_;
  bool velocity_controller_running_;
  std::chrono::system_clock::time_point begin;
  // for velocity control, store last position command
  std::vector<double> joint_pos_commands_old_;
};

typedef GenericSystem GenericRobot;

}  // namespace fake_components

#endif  // FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_
