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

#include <algorithm>
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
    update_data_from_interfaces();
    std::array<double, 3> position;
    std::copy(data_.begin(), data_.begin() + 3, position.begin());
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
    update_data_from_interfaces();
    std::array<double, 4> orientation;
    std::copy(data_.begin() + 3, data_.end(), orientation.begin());
    return orientation;
  }

  /// Fill pose message with current values.
  /**
   * Fill a pose message with current position and orientation from the state interfaces.
   */
  bool get_values_as_message(geometry_msgs::msg::Pose & message) const
  {
    update_data_from_interfaces();

    message.position.x = data_[0];
    message.position.y = data_[1];
    message.position.z = data_[2];
    message.orientation.x = data_[3];
    message.orientation.y = data_[4];
    message.orientation.z = data_[5];
    message.orientation.w = data_[6];

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

  /// Array to store the data of the pose sensor
  mutable std::array<double, 7> data_{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__POSE_SENSOR_HPP_
