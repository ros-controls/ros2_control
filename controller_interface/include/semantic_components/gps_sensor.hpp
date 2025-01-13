// Copyright 2025 ros2_control development team
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

#ifndef SEMANTIC_COMPONENTS__GPS_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__GPS_SENSOR_HPP_

#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace semantic_components
{
class GPSSensor : public SemanticComponentInterface<sensor_msgs::msg::NavSatFix>
{
public:
  explicit GPSSensor(const std::string & name)
  : SemanticComponentInterface(
      name, {{name + "/" + "status"},
             {name + "/" + "latitude"},
             {name + "/" + "longitude"},
             {name + "/" + "altitude"}})
  {
  }

  /**
   * Return GPS's status e.g. fix/no_fix
   *
   * \return Latitude.
   */
  int8_t get_status() { return static_cast<int8_t>(state_interfaces_[0].get().get_value()); }

  /**
   * Return latitude reported by a GPS
   *
   * \return Latitude.
   */
  double get_latitude() { return state_interfaces_[1].get().get_value(); }

  /**
   * Return longitude reported by a GPS
   *
   * \return Longitude.
   */
  double get_longitude() { return state_interfaces_[2].get().get_value(); }

  /**
   * Return altitude reported by a GPS
   *
   * \return Altitude.
   */
  double get_altitude() { return state_interfaces_[3].get().get_value(); }

  /**
   * Fills a NavSatFix message from the current values.
   */
  virtual bool get_values_as_message(sensor_msgs::msg::NavSatFix & message)
  {
    message.status.status = get_status();
    message.latitude = get_latitude();
    message.longitude = get_longitude();
    message.altitude = get_altitude();

    return true;
  }
};

class GPSSensorWithCovariance : public GPSSensor
{
public:
  explicit GPSSensorWithCovariance(const std::string & name) : GPSSensor(name)
  {
    interface_names_.emplace_back(name + "/" + "latitude_covariance");
    interface_names_.emplace_back(name + "/" + "longitude_covariance");
    interface_names_.emplace_back(name + "/" + "altitude_covariance");
  }

  /**
   * Return covariance reported by a GPS.
   *
   * \return Covariance array.
   */
  std::array<double, 9> get_covariance()
  {
    return std::array<double, 9>(
      {{state_interfaces_[4].get().get_value(), 0.0, 0.0, 0.0,
        state_interfaces_[5].get().get_value(), 0.0, 0.0, 0.0,
        state_interfaces_[6].get().get_value()}});
  }

  /**
   * Fills a NavSatFix message with the current values,
   * including the diagonal elements of the position covariance.
   * \return true
   */
  bool get_values_as_message(sensor_msgs::msg::NavSatFix & message) override
  {
    GPSSensor::get_values_as_message(message);
    message.position_covariance = get_covariance();
    return true;
  }
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__GPS_SENSOR_HPP_
