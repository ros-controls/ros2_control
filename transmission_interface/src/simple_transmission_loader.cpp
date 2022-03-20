// Copyright 2022 PAL Robotics S.L.
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

#include "transmission_interface/simple_transmission_loader.hpp"

#include <memory>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/simple_transmission.hpp"

namespace transmission_interface
{
std::shared_ptr<Transmission> SimpleTransmissionLoader::load(
  const hardware_interface::TransmissionInfo & transmission_info)
{
  try
  {
    const auto mechanical_reduction = transmission_info.joints.at(0).mechanical_reduction;
    const auto offset = transmission_info.joints.at(0).offset;
    std::shared_ptr<Transmission> transmission(
      new SimpleTransmission(mechanical_reduction, offset));
    return transmission;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("simple_transmission_loader"), "Failed to construct transmission '%s'",
      ex.what());
    return std::shared_ptr<Transmission>();
  }
}

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(
  transmission_interface::SimpleTransmissionLoader, transmission_interface::TransmissionLoader)
