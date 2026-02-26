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
#include <memory>
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
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
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

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    const std::string & sensor_name = get_hardware_info().sensors[0].name;
    orientation_x_ = std::make_shared<StateInterface>(sensor_name, "orientation.x");
    orientation_y_ = std::make_shared<StateInterface>(sensor_name, "orientation.y");
    orientation_z_ = std::make_shared<StateInterface>(sensor_name, "orientation.z");
    orientation_w_ = std::make_shared<StateInterface>(sensor_name, "orientation.w");
    angular_velocity_x_ = std::make_shared<StateInterface>(sensor_name, "angular_velocity.x");
    angular_velocity_y_ = std::make_shared<StateInterface>(sensor_name, "angular_velocity.y");
    angular_velocity_z_ = std::make_shared<StateInterface>(sensor_name, "angular_velocity.z");
    linear_acceleration_x_ = std::make_shared<StateInterface>(sensor_name, "linear_acceleration.x");
    linear_acceleration_y_ = std::make_shared<StateInterface>(sensor_name, "linear_acceleration.y");
    linear_acceleration_z_ = std::make_shared<StateInterface>(sensor_name, "linear_acceleration.z");
    return {orientation_x_,        orientation_y_,         orientation_z_,
            orientation_w_,        angular_velocity_x_,    angular_velocity_y_,
            angular_velocity_z_,   linear_acceleration_x_, linear_acceleration_y_,
            linear_acceleration_z_};
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // generate a random distribution of the quaternion
    std::uniform_real_distribution<double> distribution_1(0.0, 1.0);
    const double u1 = distribution_1(generator_);
    const double u2 = distribution_1(generator_);
    const double u3 = distribution_1(generator_);
    (void)orientation_w_->set_value(std::sqrt(1. - u1) * std::sin(2 * M_PI * u2), true);
    (void)orientation_x_->set_value(std::sqrt(1. - u1) * std::cos(2 * M_PI * u2), true);
    (void)orientation_y_->set_value(std::sqrt(u1) * std::sin(2 * M_PI * u3), true);
    (void)orientation_z_->set_value(std::sqrt(u1) * std::cos(2 * M_PI * u3), true);

    // generate random angular velocities and linear accelerations
    std::uniform_real_distribution<double> distribution_2(0.0, 0.1);
    (void)angular_velocity_x_->set_value(distribution_2(generator_), true);
    (void)angular_velocity_y_->set_value(distribution_2(generator_), true);
    (void)angular_velocity_z_->set_value(distribution_2(generator_), true);

    (void)linear_acceleration_x_->set_value(distribution_2(generator_), true);
    (void)linear_acceleration_y_->set_value(distribution_2(generator_), true);
    (void)linear_acceleration_z_->set_value(distribution_2(generator_), true);
    return return_type::OK;
  }

private:
  std::default_random_engine generator_;
  StateInterface::SharedPtr orientation_x_;
  StateInterface::SharedPtr orientation_y_;
  StateInterface::SharedPtr orientation_z_;
  StateInterface::SharedPtr orientation_w_;
  StateInterface::SharedPtr angular_velocity_x_;
  StateInterface::SharedPtr angular_velocity_y_;
  StateInterface::SharedPtr angular_velocity_z_;
  StateInterface::SharedPtr linear_acceleration_x_;
  StateInterface::SharedPtr linear_acceleration_y_;
  StateInterface::SharedPtr linear_acceleration_z_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(test_hardware_components::TestIMUSensor, hardware_interface::SensorInterface)
