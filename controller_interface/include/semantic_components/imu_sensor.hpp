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
  explicit IMUSensor(const std::string & name) : SemanticComponentInterface(name, 10)
  {
    interface_names_.emplace_back(name_ + "/" + "orientation.x");
    interface_names_.emplace_back(name_ + "/" + "orientation.y");
    interface_names_.emplace_back(name_ + "/" + "orientation.z");
    interface_names_.emplace_back(name_ + "/" + "orientation.w");
    interface_names_.emplace_back(name_ + "/" + "angular_velocity.x");
    interface_names_.emplace_back(name_ + "/" + "angular_velocity.y");
    interface_names_.emplace_back(name_ + "/" + "angular_velocity.z");
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.x");
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.y");
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.z");

    // Set default values to NaN
    orientation_.fill(std::numeric_limits<double>::quiet_NaN());
    angular_velocity_.fill(std::numeric_limits<double>::quiet_NaN());
    linear_acceleration_.fill(std::numeric_limits<double>::quiet_NaN());
  }

  virtual ~IMUSensor() = default;

  /// Return orientation.
  /**
   * Return orientation reported by an IMU
   *
   * \return array of size 4 with orientation quaternion (x,y,z,w)
   */
  std::array<double, 4> get_orientation()
  {
    size_t interface_offset = 0;
    for (size_t i = 0; i < orientation_.size(); ++i)
    {
      orientation_[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return orientation_;
  }

  /// Return angular velocity.
  /**
   * Return angular velocity reported by an IMU
   *
   * \return array of size 3 with angular velocity values.
   */
  std::array<double, 3> get_angular_velocity()
  {
    size_t interface_offset = orientation_.size();
    for (size_t i = 0; i < angular_velocity_.size(); ++i)
    {
      angular_velocity_[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return angular_velocity_;
  }

  /// Return linear acceleration.
  /**
   * Return linear acceleration reported by an IMU
   *
   * \return array of size 3 with linear acceleration values.
   */
  std::array<double, 3> get_linear_acceleration()
  {
    size_t interface_offset = orientation_.size() + angular_velocity_.size();
    for (size_t i = 0; i < linear_acceleration_.size(); ++i)
    {
      linear_acceleration_[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return linear_acceleration_;
  }

  /// Return Imu message with orientation, angular velocity and linear acceleration
  /**
   * Constructs and return a IMU message from the current values.
   * \return imu message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Imu & message)
  {
    // call get_orientation() and get_angular_velocity()  get_linear_acceleration() to
    // update with the latest values
    get_orientation();
    get_angular_velocity();
    get_linear_acceleration();

    // update the message values, covariances unknown
    message.orientation.x = orientation_[0];
    message.orientation.y = orientation_[1];
    message.orientation.z = orientation_[2];
    message.orientation.w = orientation_[3];

    message.angular_velocity.x = angular_velocity_[0];
    message.angular_velocity.y = angular_velocity_[1];
    message.angular_velocity.z = angular_velocity_[2];

    message.linear_acceleration.x = linear_acceleration_[0];
    message.linear_acceleration.y = linear_acceleration_[1];
    message.linear_acceleration.z = linear_acceleration_[2];

    return true;
  }

protected:
  // Order is: orientation X,Y,Z,W angular velocity X,Y,Z and linear acceleration X,Y,Z
  std::array<double, 4> orientation_;
  std::array<double, 3> angular_velocity_;
  std::array<double, 3> linear_acceleration_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__IMU_SENSOR_HPP_
