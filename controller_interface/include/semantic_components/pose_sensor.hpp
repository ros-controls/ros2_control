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
  explicit PoseSensor(const std::string & name)
  : SemanticComponentInterface(
      name, {{name + '/' + "position.x"},
             {name + '/' + "position.y"},
             {name + '/' + "position.z"},
             {name + '/' + "orientation.x"},
             {name + '/' + "orientation.y"},
             {name + '/' + "orientation.z"},
             {name + '/' + "orientation.w"}})
  {
  }
  /// Update and return position.
  /*!
   * Update and return current pose position from state interfaces.
   *
   * \return Array of position coordinates.
   */
  std::array<double, 3> get_position() const
  {
    std::array<double, 3> position;
    for (auto i = 0u; i < position.size(); ++i)
    {
      position[i] = state_interfaces_[i].get().get_value();
    }
    return position;
  }

  /// Update and return orientation
  /*!
   * Update and return current pose orientation from state interfaces.
   *
   * \return Array of orientation coordinates in xyzw convention.
   */
  std::array<double, 4> get_orientation() const
  {
    std::array<double, 4> orientation;
    const std::size_t interface_offset{3};
    for (auto i = 0u; i < orientation.size(); ++i)
    {
      orientation[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    return orientation;
  }

  /// Fill pose message with current values.
  /**
   * Fill a pose message with current position and orientation from the state interfaces.
   */
  bool get_values_as_message(geometry_msgs::msg::Pose & message) const
  {
    const auto [position_x, position_y, position_z] = get_position();
    const auto [orientation_x, orientation_y, orientation_z, orientation_w] = get_orientation();

    message.position.x = position_x;
    message.position.y = position_y;
    message.position.z = position_z;
    message.orientation.x = orientation_x;
    message.orientation.y = orientation_y;
    message.orientation.z = orientation_z;
    message.orientation.w = orientation_w;

    return true;
  }
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__POSE_SENSOR_HPP_
