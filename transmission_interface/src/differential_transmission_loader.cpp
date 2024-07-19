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

#include "transmission_interface/differential_transmission_loader.hpp"

#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/differential_transmission.hpp"

namespace transmission_interface
{
std::shared_ptr<Transmission> DifferentialTransmissionLoader::load(
  const hardware_interface::TransmissionInfo & transmission_info)
{
  try
  {
    const auto act_reduction1 = transmission_info.actuators.at(0).mechanical_reduction;
    const auto act_reduction2 = transmission_info.actuators.at(1).mechanical_reduction;

    const auto jnt_reduction1 = transmission_info.joints.at(0).mechanical_reduction;
    const auto jnt_reduction2 = transmission_info.joints.at(1).mechanical_reduction;

    const auto jnt_offset1 = transmission_info.joints.at(0).offset;
    const auto jnt_offset2 = transmission_info.joints.at(1).offset;

    std::shared_ptr<Transmission> transmission(new DifferentialTransmission(
      {act_reduction1, act_reduction2}, {jnt_reduction1, jnt_reduction2},
      {jnt_offset1, jnt_offset2}));
    return transmission;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("differential_transmission_loader"),
      "Failed to construct transmission '%s'", ex.what());
    return std::shared_ptr<Transmission>();
  }
}

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(
  transmission_interface::DifferentialTransmissionLoader,
  transmission_interface::TransmissionLoader)
