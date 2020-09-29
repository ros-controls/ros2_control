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

#ifndef TEST_HARDWARE_RESOURCES__TEST_SYSTEM_HARDWARE_HPP_
#define TEST_HARDWARE_RESOURCES__TEST_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/hardware_resources/system_hardware_interface.hpp"

class TestSystemHardware : public hardware_interface::hardware_resources::SystemHardwareInterface
{
public:
  hardware_interface::return_type
  configure(const hardware_interface::hardware_resources::HardwareInfo & system_info) override;

  hardware_interface::return_type
  start() override;

  hardware_interface::return_type
  stop() override;

  hardware_interface::hardware_interface_status
  get_status() const override;

  hardware_interface::return_type
  read_sensors(
    std::vector<std::shared_ptr<hardware_interface::components::Sensor>> & sensors) const override;

  hardware_interface::return_type
  read_joints(
    std::vector<std::shared_ptr<hardware_interface::components::Joint>> & joints) const override;

  hardware_interface::return_type
  write_joints(
    const std::vector<std::shared_ptr<hardware_interface::components::Joint>> & joints) override;
};

#endif  // TEST_HARDWARE_RESOURCES__TEST_SYSTEM_HARDWARE_HPP_
