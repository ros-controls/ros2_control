///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros2_control
#include "transmission_interface/simple_transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace transmission_interface
{

std::shared_ptr<Transmission> SimpleTransmissionLoader::load(const hardware_interface::TransmissionInfo& transmission_info)
{

  // Transmission instance
  try
  {
    std::shared_ptr<Transmission> transmission(new SimpleTransmission(transmission_info.joints[0].mechanical_reduction, transmission_info.joints[0].offset));
    return transmission;
  }
  catch(const transmission_interface::TransmissionInterfaceException & ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("simple_transmission_loader"), "Failed to construct transmission '%s'", ex.what());
    return std::shared_ptr<Transmission>();
  }
  
}


} // namespace


PLUGINLIB_EXPORT_CLASS(transmission_interface::SimpleTransmissionLoader,
                       transmission_interface::TransmissionLoader)