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
#include <stdexcept>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/logging.hpp"

using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;

class TestSensor : public SensorInterface
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    // can only give feedback state for velocity
    if (get_hardware_info().sensors[0].state_interfaces.size() == 2)
    {
      return CallbackReturn::ERROR;
    }
    if (get_hardware_info().rw_rate == 0u)
    {
      RCLCPP_WARN(
        get_logger(),
        "Sensor hardware component '%s' from plugin '%s' failed to initialize as rw_rate is 0.",
        get_hardware_info().name.c_str(), get_hardware_info().hardware_plugin_name.c_str());
      return CallbackReturn::ERROR;
    }

    auto it = get_hardware_info().hardware_parameters.find("throw_on_read");
    if (it != get_hardware_info().hardware_parameters.end())
    {
      throw_on_read_ = (it->second == "true");
    }
    it = get_hardware_info().hardware_parameters.find("throw_on_configure");
    if (it != get_hardware_info().hardware_parameters.end())
    {
      throw_on_configure_ = (it->second == "true");
    }
    it = get_hardware_info().hardware_parameters.find("throw_on_activate");
    if (it != get_hardware_info().hardware_parameters.end())
    {
      throw_on_activate_ = (it->second == "true");
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (throw_on_configure_)
    {
      throw std::runtime_error("Injected exception during on_configure!");
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (throw_on_activate_)
    {
      throw std::runtime_error("Injected exception during on_activate!");
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    velocity_state_ = std::make_shared<StateInterface>(
      get_hardware_info().sensors[0].name, get_hardware_info().sensors[0].state_interfaces[0].name);
    std::ignore = velocity_state_->set_value(0.0, false);
    state_interfaces.push_back(velocity_state_);

    return state_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (throw_on_read_)
    {
      throw std::runtime_error("Exception from TestSensor::read() as requested by parameter.");
    }
    return return_type::OK;
  }

private:
  bool throw_on_read_ = false;
  bool throw_on_configure_ = false;
  bool throw_on_activate_ = false;
  StateInterface::SharedPtr velocity_state_;
};

class TestUninitializableSensor : public TestSensor
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    SensorInterface::on_init(params);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestSensor, hardware_interface::SensorInterface)
PLUGINLIB_EXPORT_CLASS(TestUninitializableSensor, hardware_interface::SensorInterface)
