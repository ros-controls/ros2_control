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

#include <array>
#include <limits>
#include <string>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace semantic_components
{

enum class GPSSensorOption
{
  WithCovariance,
  WithoutCovariance
};

template <GPSSensorOption sensor_option>
class GPSSensor : public SemanticComponentInterface<sensor_msgs::msg::NavSatFix>
{
public:
  static_assert(
    sensor_option == GPSSensorOption::WithCovariance ||
      sensor_option == GPSSensorOption::WithoutCovariance,
    "Invalid GPSSensorOption");
  explicit GPSSensor(const std::string & name)
  : SemanticComponentInterface(
      name, {{name + "/" + "status"},
             {name + "/" + "service"},
             {name + "/" + "latitude"},
             {name + "/" + "longitude"},
             {name + "/" + "altitude"}})
  {
    if constexpr (sensor_option == GPSSensorOption::WithCovariance)
    {
      interface_names_.emplace_back(name + "/" + "latitude_covariance");
      interface_names_.emplace_back(name + "/" + "longitude_covariance");
      interface_names_.emplace_back(name + "/" + "altitude_covariance");
    }
  }

  /**
   * Return GPS's status e.g. fix/no_fix
   *
   * \return Status
   */
  int8_t get_status() const
  {
    const auto data = state_interfaces_[0].get().get_optional();
    if (data.has_value())
    {
      return static_cast<int8_t>(data.value());
    }
    return std::numeric_limits<int8_t>::max();
  }

  /**
   * Return service used by GPS e.g. fix/no_fix
   *
   * \return Service
   */
  uint16_t get_service() const
  {
    const auto data = state_interfaces_[1].get().get_optional();
    if (data.has_value())
    {
      return static_cast<uint16_t>(data.value());
    }
    return std::numeric_limits<uint16_t>::max();
  }

  /**
   * Return latitude reported by a GPS
   *
   * \return Latitude.
   */
  double get_latitude() const
  {
    const auto data = state_interfaces_[2].get().get_optional();
    if (data.has_value())
    {
      return data.value();
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * Return longitude reported by a GPS
   *
   * \return Longitude.
   */
  double get_longitude() const
  {
    const auto data = state_interfaces_[3].get().get_optional();
    if (data.has_value())
    {
      return data.value();
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * Return altitude reported by a GPS
   *
   * \return Altitude.
   */
  double get_altitude() const
  {
    const auto data = state_interfaces_[4].get().get_optional();
    if (data.has_value())
    {
      return data.value();
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * Return covariance reported by a GPS.
   *
   * \return Covariance array.
   */
  template <
    typename U = void,
    typename = std::enable_if_t<sensor_option == GPSSensorOption::WithCovariance, U>>
  const std::array<double, 9> & get_covariance()
  {
    const auto data_1 = state_interfaces_[5].get().get_optional();
    const auto data_2 = state_interfaces_[6].get().get_optional();
    const auto data_3 = state_interfaces_[7].get().get_optional();
    if (data_1.has_value())
    {
      covariance_[0] = data_1.value();
    }
    if (data_2.has_value())
    {
      covariance_[4] = data_2.value();
    }
    if (data_3.has_value())
    {
      covariance_[8] = data_3.value();
    }
    return covariance_;
  }

  /**
   * Fills a NavSatFix message from the current values.
   */
  bool get_values_as_message(sensor_msgs::msg::NavSatFix & message)
  {
    message.status.status = get_status();
    message.status.service = get_service();
    message.latitude = get_latitude();
    message.longitude = get_longitude();
    message.altitude = get_altitude();

    if constexpr (sensor_option == GPSSensorOption::WithCovariance)
    {
      message.position_covariance = get_covariance();
    }

    return true;
  }

private:
  std::array<double, 9> covariance_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__GPS_SENSOR_HPP_
