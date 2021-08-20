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

#include <memory>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"

using hardware_interface::BaseInterface;
using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;
using hardware_interface::status;

class TestSensor : public BaseInterface<SensorInterface>
{
  return_type configure(const hardware_interface::HardwareInfo & info) override
  {
    if (configure_default(info) != return_type::OK)
    {
      return return_type::ERROR;
    }
    // can only give feedback state for velocity
    if (info_.sensors[0].state_interfaces.size() != 1)
    {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &velocity_state_));

    return state_interfaces;
  }

  return_type start() override
  {
    status_ = status::STARTED;
    return return_type::OK;
  }

  return_type stop() override
  {
    status_ = status::STOPPED;
    return return_type::OK;
  }

  return_type read() override { return return_type::OK; }

private:
  double velocity_state_ = 0.0;
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestSensor, hardware_interface::SensorInterface)
