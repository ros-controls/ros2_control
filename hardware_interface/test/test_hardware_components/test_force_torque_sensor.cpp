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

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;

namespace test_hardware_components
{
class TestForceTorqueSensor : public SensorInterface
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    const auto & state_interfaces = get_hardware_info().sensors[0].state_interfaces;
    if (state_interfaces.size() != 6)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & ft_key : {"fx", "fy", "fz", "tx", "ty", "tz"})
    {
      if (
        std::find_if(
          state_interfaces.begin(), state_interfaces.end(), [&ft_key](const auto & interface_info)
          { return interface_info.name == ft_key; }) == state_interfaces.end())
      {
        return CallbackReturn::ERROR;
      }
    }

    fprintf(stderr, "TestForceTorqueSensor configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    const auto & sensor_name = get_hardware_info().sensors[0].name;
    fx_interface_ = std::make_shared<StateInterface>(sensor_name, "fx");
    fy_interface_ = std::make_shared<StateInterface>(sensor_name, "fy");
    fz_interface_ = std::make_shared<StateInterface>(sensor_name, "fz");
    tx_interface_ = std::make_shared<StateInterface>(sensor_name, "tx");
    ty_interface_ = std::make_shared<StateInterface>(sensor_name, "ty");
    tz_interface_ = std::make_shared<StateInterface>(sensor_name, "tz");
    return {fx_interface_, fy_interface_, fz_interface_,
            tx_interface_, ty_interface_, tz_interface_};
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    double fx = 0.0, fy = 0.0, fz = 0.0, tx = 0.0, ty = 0.0, tz = 0.0;
    (void)fx_interface_->get_value(fx, true);
    (void)fy_interface_->get_value(fy, true);
    (void)fz_interface_->get_value(fz, true);
    (void)tx_interface_->get_value(tx, true);
    (void)ty_interface_->get_value(ty, true);
    (void)tz_interface_->get_value(tz, true);
    (void)fx_interface_->set_value(fmod((fx + 1.0), 10), true);
    (void)fy_interface_->set_value(fmod((fy + 1.0), 10), true);
    (void)fz_interface_->set_value(fmod((fz + 1.0), 10), true);
    (void)tx_interface_->set_value(fmod((tx + 1.0), 10), true);
    (void)ty_interface_->set_value(fmod((ty + 1.0), 10), true);
    (void)tz_interface_->set_value(fmod((tz + 1.0), 10), true);
    return return_type::OK;
  }

private:
  StateInterface::SharedPtr fx_interface_;
  StateInterface::SharedPtr fy_interface_;
  StateInterface::SharedPtr fz_interface_;
  StateInterface::SharedPtr tx_interface_;
  StateInterface::SharedPtr ty_interface_;
  StateInterface::SharedPtr tz_interface_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestForceTorqueSensor, hardware_interface::SensorInterface)
