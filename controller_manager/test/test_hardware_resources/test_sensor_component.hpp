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

#ifndef TEST_HARDWARE_RESOURCES__TEST_SENSOR_COMPONENT_HPP_
#define TEST_HARDWARE_RESOURCES__TEST_SENSOR_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_manager/visibility_control.h"

#include "hardware_interface/components/sensor_interface.hpp"

class TestSensorComponent : public hardware_interface::components::SensorInterface
{
public:
  TestSensorComponent() = default;

  virtual ~TestSensorComponent() = default;

  CONTROLLER_MANAGER_PUBLIC
  hardware_interface::return_type
  configure(const hardware_interface::components::ComponentInfo & sensor_info) override;

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> get_state_interfaces() const override;

  CONTROLLER_MANAGER_PUBLIC
  hardware_interface::return_type get_state(
    std::vector<double> & state,
    const std::vector<std::string> & interfaces) const override;

  CONTROLLER_MANAGER_PUBLIC
  hardware_interface::return_type get_state(std::vector<double> & state) const override;

  CONTROLLER_MANAGER_PUBLIC
  hardware_interface::return_type set_state(
    const std::vector<double> & state,
    const std::vector<std::string> & interfaces) override;

  CONTROLLER_MANAGER_PUBLIC
  hardware_interface::return_type set_state(const std::vector<double> & state) override;

  // Some additional API
  bool return_true() const {return true;}
};

#endif  // TEST_HARDWARE_RESOURCES__TEST_SENSOR_COMPONENT_HPP_
