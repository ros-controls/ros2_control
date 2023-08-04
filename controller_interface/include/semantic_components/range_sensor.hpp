// Copyright 2023 flochre
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

#ifndef SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace semantic_components
{
class RangeSensor : public SemanticComponentInterface<sensor_msgs::msg::Range>
{
public:
  explicit RangeSensor(const std::string & name) : SemanticComponentInterface(name, 1)
  {
    interface_names_.emplace_back(name_ + "/" + "range");
  }

  virtual ~RangeSensor() = default;

  /**
   * Return Range reported by a sensor
   *
   * \return value of the range in meters
   */
  float get_range() { return state_interfaces_[0].get().get_value(); }

  /// Return Range message with range in meters
  /**
   * Constructs and return a Range message from the current values.
   * \return Range message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Range & message)
  {
    // update the message values
    message.range = get_range();

    return true;
  }
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_
