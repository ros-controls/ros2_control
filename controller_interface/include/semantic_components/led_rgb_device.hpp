// Copyright (c) 2024, Sherpa Mobile Robotics
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

#ifndef SEMANTIC_COMPONENTS__LED_RGB_DEVICE_HPP_
#define SEMANTIC_COMPONENTS__LED_RGB_DEVICE_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_command_interface.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace semantic_components
{
class LEDRgbDevice : public SemanticComponentCommandInterface<std_msgs::msg::ColorRGBA>
{
public:
  /**
   * Constructor for a LED RGB device with interface names set based on device name.
   * The constructor sets the command interface names to "<name>/r", "<name>/g", "<name>/b".
   */
  explicit LEDRgbDevice(const std::string & name) : SemanticComponentCommandInterface(name, 3)
  {
    interface_names_.emplace_back(name_ + "/" + "r");
    interface_names_.emplace_back(name_ + "/" + "g");
    interface_names_.emplace_back(name_ + "/" + "b");
  }

  /**
   * Constructor for a LED RGB device with custom interface names.
   * The constructor takes the three command interface names for the red, green, and blue channels.
   */
  explicit LEDRgbDevice(
    const std::string & interface_r, const std::string & interface_g,
    const std::string & interface_b)
  : SemanticComponentCommandInterface("", 3)
  {
    interface_names_.emplace_back(interface_r);
    interface_names_.emplace_back(interface_g);
    interface_names_.emplace_back(interface_b);
  }

  virtual ~LEDRgbDevice() = default;

  /// Set LED states from ColorRGBA message
  bool set_values_from_message(std_msgs::msg::ColorRGBA & message)
  {
    if (
      message.r < 0 || message.r > 1 || message.g < 0 || message.g > 1 || message.b < 0 ||
      message.b > 1)
    {
      return false;
    }
    bool all_set = true;
    all_set &= command_interfaces_[0].get().set_value(message.r);
    all_set &= command_interfaces_[1].get().set_value(message.g);
    all_set &= command_interfaces_[2].get().set_value(message.b);
    return all_set;
  }
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__LED_RGB_DEVICE_HPP_
