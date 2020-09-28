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

#include "hardware_interface/components/sensor.hpp"

class TestSensorComponent : public hardware_interface::components::Sensor
{
public:
  hardware_interface::return_type
  configure(const hardware_interface::components::ComponentInfo & sensor_info) override;

  std::vector<std::string> get_state_interfaces() const override;
};

#endif  // TEST_HARDWARE_RESOURCES__TEST_SENSOR_COMPONENT_HPP_
