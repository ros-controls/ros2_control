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

#include <algorithm>
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
    update_data_from_interfaces();
    std::array<double, 4> orientation;
    std::copy(data_.begin(), data_.begin() + 4, orientation.begin());
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
    update_data_from_interfaces();
    std::array<double, 3> angular_velocity;
    std::copy(data_.begin() + 4, data_.begin() + 7, angular_velocity.begin());
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
    update_data_from_interfaces();
    std::array<double, 3> linear_acceleration;
    std::copy(data_.begin() + 7, data_.end(), linear_acceleration.begin());
    return linear_acceleration;
  }

  /// Return Imu message with orientation, angular velocity and linear acceleration
  /**
   * Constructs and return a IMU message from the current values.
   * \return imu message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Imu & message) const
  {
    update_data_from_interfaces();
    message.orientation.x = data_[0];
    message.orientation.y = data_[1];
    message.orientation.z = data_[2];
    message.orientation.w = data_[3];

    message.angular_velocity.x = data_[4];
    message.angular_velocity.y = data_[5];
    message.angular_velocity.z = data_[6];

    message.linear_acceleration.x = data_[7];
    message.linear_acceleration.y = data_[8];
    message.linear_acceleration.z = data_[9];

    return true;
  }

private:
  /**
   * @brief Update the data array from the state interfaces.
   * @note This method is thread-safe and non-blocking.
   * @note This method might return stale data if the data is not updated. This is to ensure that
   * the data from the sensor is not discontinuous.
   */
  void update_data_from_interfaces() const
  {
    for (auto i = 0u; i < data_.size(); ++i)
    {
      const auto data = state_interfaces_[i].get().get_optional();
      if (data.has_value())
      {
        data_[i] = data.value();
      }
    }
  }

  // Array to store the data of the IMU sensor
  mutable std::array<double, 10> data_{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__IMU_SENSOR_HPP_
