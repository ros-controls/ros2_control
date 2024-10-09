// Copyright 2024 FZI Forschungszentrum Informatik
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
#ifndef SEMANTIC_COMPONENTS__POSE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__POSE_SENSOR_HPP_

#include <array>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components
{

class PoseSensor : public SemanticComponentInterface<geometry_msgs::msg::Pose>
{
public:
  /// Constructor for a standard pose sensor with interface names set based on sensor name.
  explicit PoseSensor(const std::string & name) : SemanticComponentInterface{name, 7}
  {
    // Use standard interface names
    interface_names_.emplace_back(name_ + '/' + "position.x");
    interface_names_.emplace_back(name_ + '/' + "position.y");
    interface_names_.emplace_back(name_ + '/' + "position.z");
    interface_names_.emplace_back(name_ + '/' + "orientation.x");
    interface_names_.emplace_back(name_ + '/' + "orientation.y");
    interface_names_.emplace_back(name_ + '/' + "orientation.z");
    interface_names_.emplace_back(name_ + '/' + "orientation.w");

    // Set all sensor values to default value NaN
    std::fill(position_.begin(), position_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(orientation_.begin(), orientation_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  virtual ~PoseSensor() = default;

  /// Update and return position.
  /*!
   * Update and return current pose position from state interfaces.
   *
   * \return Array of position coordinates.
   */
  std::array<double, 3> get_position()
  {
    for (size_t i = 0; i < 3; ++i)
    {
      position_[i] = state_interfaces_[i].get().get_value();
    }

    return position_;
  }

  /// Update and return orientation
  /*!
   * Update and return current pose orientation from state interfaces.
   *
   * \return Array of orientation coordinates in xyzw convention.
   */
  std::array<double, 4> get_orientation()
  {
    for (size_t i = 3; i < 7; ++i)
    {
      orientation_[i - 3] = state_interfaces_[i].get().get_value();
    }

    return orientation_;
  }

  /// Fill pose message with current values.
  /**
   * Fill a pose message with current position and orientation from the state interfaces.
   */
  bool get_values_as_message(geometry_msgs::msg::Pose & message)
  {
    // Update state from state interfaces
    get_position();
    get_orientation();

    // Set message values from current state
    message.position.x = position_[0];
    message.position.y = position_[1];
    message.position.z = position_[2];
    message.orientation.x = orientation_[0];
    message.orientation.y = orientation_[1];
    message.orientation.z = orientation_[2];
    message.orientation.w = orientation_[3];

    return true;
  }

protected:
  std::array<double, 3> position_;
  std::array<double, 4> orientation_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__POSE_SENSOR_HPP_
