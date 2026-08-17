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

#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/logging.hpp"

using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;

class TestSensor : public SensorInterface
{
  void check_injected_failure(const std::string & method_name) const
  {
    const auto & info = get_hardware_info();
    const auto & params = info.hardware_parameters;
    const std::string throw_key = "throw_on_" + method_name;
    const std::string unknown_throw_key = "throw_unknown_on_" + method_name;

    if (params.count(throw_key) && hardware_interface::parse_bool(params.at(throw_key)))
    {
      throw std::runtime_error("Standard exception from TestSensor::" + method_name);
    }

    if (
      params.count(unknown_throw_key) &&
      hardware_interface::parse_bool(params.at(unknown_throw_key)))
    {
      RCLCPP_DEBUG(
        get_logger(), "Injecting unknown exception for TestSensor::%s", method_name.c_str());
      throw 42;  // Throw an unknown type (int)
    }
  }

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    check_injected_failure("on_init");
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
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_configure");
    return SensorInterface::on_configure(previous_state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_cleanup");
    return SensorInterface::on_cleanup(previous_state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_activate");
    return SensorInterface::on_activate(previous_state);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_deactivate");
    return SensorInterface::on_deactivate(previous_state);
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_shutdown");
    return SensorInterface::on_shutdown(previous_state);
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override
  {
    check_injected_failure("on_error");
    return SensorInterface::on_error(previous_state);
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    check_injected_failure("export_state_interfaces");
    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    velocity_state_ = std::make_shared<StateInterface>(
      get_hardware_info().sensors[0].name, get_hardware_info().sensors[0].state_interfaces[0].name);
    std::ignore = velocity_state_->set_value(0.0, false);
    state_interfaces.push_back(velocity_state_);

    return state_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    check_injected_failure("read");
    return return_type::OK;
  }

private:
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
