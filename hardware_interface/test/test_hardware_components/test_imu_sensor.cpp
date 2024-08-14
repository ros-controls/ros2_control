// Copyright 2023 PAL Robotics SL.
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

/*
 * Author: Sai Kishor Kothakota
 */

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;

namespace test_hardware_components
{
class TestIMUSensor : public SensorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & sensor_info) override
  {
    if (SensorInterface::on_init(sensor_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    const auto & state_interfaces = get_hardware_info().sensors[0].state_interfaces;
    if (state_interfaces.size() != 10)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & imu_key :
         {"orientation.x", "orientation.y", "orientation.z", "orientation.w", "angular_velocity.x",
          "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x",
          "linear_acceleration.y", "linear_acceleration.z"})
    {
      if (
        std::find_if(
          state_interfaces.begin(), state_interfaces.end(), [&imu_key](const auto & interface_info)
          { return interface_info.name == imu_key; }) == state_interfaces.end())
      {
        return CallbackReturn::ERROR;
      }
    }

    fprintf(stderr, "TestIMUSensor configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;

    const std::string & sensor_name = get_hardware_info().sensors[0].name;
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "orientation.x", &orientation_.x));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "orientation.y", &orientation_.y));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "orientation.z", &orientation_.z));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "orientation.w", &orientation_.w));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "angular_velocity.x", &angular_velocity_.x));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "angular_velocity.y", &angular_velocity_.y));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "angular_velocity.z", &angular_velocity_.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      sensor_name, "linear_acceleration.x", &linear_acceleration_.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      sensor_name, "linear_acceleration.y", &linear_acceleration_.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      sensor_name, "linear_acceleration.z", &linear_acceleration_.z));

    return state_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // generate a random distribution of the quaternion
    std::uniform_real_distribution<double> distribution_1(0.0, 1.0);
    const double u1 = distribution_1(generator_);
    const double u2 = distribution_1(generator_);
    const double u3 = distribution_1(generator_);
    orientation_.w = std::sqrt(1. - u1) * std::sin(2 * M_PI * u2);
    orientation_.x = std::sqrt(1. - u1) * std::cos(2 * M_PI * u2);
    orientation_.y = std::sqrt(u1) * std::sin(2 * M_PI * u3);
    orientation_.z = std::sqrt(u1) * std::cos(2 * M_PI * u3);

    // generate random angular velocities and linear accelerations
    std::uniform_real_distribution<double> distribution_2(0.0, 0.1);
    angular_velocity_.x = distribution_2(generator_);
    angular_velocity_.y = distribution_2(generator_);
    angular_velocity_.z = distribution_2(generator_);

    linear_acceleration_.x = distribution_2(generator_);
    linear_acceleration_.y = distribution_2(generator_);
    linear_acceleration_.z = distribution_2(generator_);
    return return_type::OK;
  }

private:
  struct QuaternionValues
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };
  struct AxisValues
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  std::default_random_engine generator_;
  QuaternionValues orientation_;
  AxisValues angular_velocity_;
  AxisValues linear_acceleration_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(test_hardware_components::TestIMUSensor, hardware_interface::SensorInterface)
