// Copyright 2021 PAL Robotics SL.
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

#ifndef SEMANTIC_COMPONENTS__IMU_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__IMU_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace semantic_components
{
class IMUSensor : public SemanticComponentInterface<sensor_msgs::msg::Imu>
{
public:
  explicit IMUSensor(const std::string & name)
  : SemanticComponentInterface(
      name, {{name + "/" + "orientation.x"},
             {name + "/" + "orientation.y"},
             {name + "/" + "orientation.z"},
             {name + "/" + "orientation.w"},
             {name + "/" + "angular_velocity.x"},
             {name + "/" + "angular_velocity.y"},
             {name + "/" + "angular_velocity.z"},
             {name + "/" + "linear_acceleration.x"},
             {name + "/" + "linear_acceleration.y"},
             {name + "/" + "linear_acceleration.z"}})
  {
  }
  /// Return orientation.
  /**
   * Return orientation reported by an IMU
   *
   * \return Array of size 4 with orientation quaternion (x,y,z,w).
   */
  std::array<double, 4> get_orientation() const
  {
    std::array<double, 4> orientation;
    for (auto i = 0u; i < orientation.size(); ++i)
    {
      orientation[i] = state_interfaces_[i].get().get_value();
    }
    return orientation;
  }

  /// Return angular velocity.
  /**
   * Return angular velocity reported by an IMU
   *
   * \return array of size 3 with angular velocity values (x, y, z).
   */
  std::array<double, 3> get_angular_velocity() const
  {
    std::array<double, 3> angular_velocity;
    const std::size_t interface_offset{4};
    for (auto i = 0u; i < angular_velocity.size(); ++i)
    {
      angular_velocity[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return angular_velocity;
  }

  /// Return linear acceleration.
  /**
   * Return linear acceleration reported by an IMU
   *
   * \return array of size 3 with linear acceleration values (x, y, z).
   */
  std::array<double, 3> get_linear_acceleration() const
  {
    std::array<double, 3> linear_acceleration;
    const std::size_t interface_offset{7};
    for (auto i = 0u; i < linear_acceleration.size(); ++i)
    {
      linear_acceleration[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return linear_acceleration;
  }

  /// Return Imu message with orientation, angular velocity and linear acceleration
  /**
   * Constructs and return a IMU message from the current values.
   * \return imu message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Imu & message) const
  {
    const auto [orientation_x, orientation_y, orientation_z, orientation_w] = get_orientation();
    const auto [angular_velocity_x, angular_velocity_y, angular_velocity_z] =
      get_angular_velocity();
    const auto [linear_acceleration_x, linear_acceleration_y, linear_acceleration_z] =
      get_linear_acceleration();

    message.orientation.x = orientation_x;
    message.orientation.y = orientation_y;
    message.orientation.z = orientation_z;
    message.orientation.w = orientation_w;

    message.angular_velocity.x = angular_velocity_x;
    message.angular_velocity.y = angular_velocity_y;
    message.angular_velocity.z = angular_velocity_z;

    message.linear_acceleration.x = linear_acceleration_x;
    message.linear_acceleration.y = linear_acceleration_y;
    message.linear_acceleration.z = linear_acceleration_z;

    return true;
  }
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__IMU_SENSOR_HPP_
