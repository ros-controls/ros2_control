// Copyright 2020 Open Source Robotics Foundation, Inc.
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

/// Parses <tt>\<transmission\></tt> elements into corresponding structs from XML (URDF).
/**
 * \file
 * \author Dave Coleman
 */

#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_

#include <transmission_interface/visibility_control.h>
#include <transmission_interface/transmission_info.hpp>

#include <tinyxml2.h>

#include <string>
#include <vector>

namespace transmission_interface
{
/// Parses the joint elements within transmission elements of a URDF
/**
 * If parse errors occur a std::runtime_error will be thrown with a description of the problem.
 * \param[in] trans_it pointer to the current XML element being parsed
 * \param[out] joints resulting list of joints in the transmission
 * \return true if joint information for a transmission was successfully parsed.
 */
TRANSMISSION_INTERFACE_PUBLIC
std::vector<JointInfo> parse_joints(tinyxml2::XMLElement * trans_it);

/// Parses the actuator elements within transmission elements of a URDF
/**
 * If parse errors occur a std::runtime_error will be thrown with a description of the problem.
 * \param[in] trans_it pointer to the current XML element being parsed
 * \param[out] actuators resulting list of actuators in the transmission
 * \return true if actuator information for a transmission was successfully parsed.
 */
TRANSMISSION_INTERFACE_PUBLIC
std::vector<ActuatorInfo> parse_actuators(tinyxml2::XMLElement * trans_it);

/// Parse transmission information from a URDF.
/**
 * \param[in] urdf A string containing the URDF xml
 * \return parsed transmission information
 * \throws std::runtime_error on malformed or empty xml
 */
TRANSMISSION_INTERFACE_PUBLIC
std::vector<TransmissionInfo> parse_transmissions_from_urdf(const std::string & urdf);
}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_
