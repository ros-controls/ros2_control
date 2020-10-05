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

/**
 * \file
 * \brief Parses <tt>\<transmission\></tt> elements into corresponding structs from XML (URDF).
 * \author Dave Coleman
 */

#pragma once


#include <transmission_interface/transmission_info.hpp>
#include <transmission_interface/visibility_control.h>

// XML
#include <tinyxml.h>

#include <string>
#include <vector>

namespace transmission_interface
{

/** \brief Parse all transmissions specified in a URDF. */
class TransmissionParser
{
public:
  /**
   * \brief Parses the transmission elements of a URDF
   * \param[in] urdf_string XML string of a valid URDF file that contains <tt>\<transmission\></tt> elements
   * \param[out] transmissions vector of loaded transmission meta data
   * \return true if parsing was successful
   */
  static bool parse(const std::string & urdf_string, std::vector<TransmissionInfo> & transmissions);

protected:
  /**
   * \brief Parses the joint elements within transmission elements of a URDF
   * \param[in] trans_it pointer to the current XML element being parsed
   * \param[out] joints resulting list of joints in the transmission
   * \return true if successful
   */
  static bool parseJoints(TiXmlElement * trans_it, std::vector<JointInfo> & joints);

  /**
   * \brief Parses the actuator elements within transmission elements of a URDF
   * \param[in] trans_it pointer to the current XML element being parsed
   * \param[out] actuators resulting list of actuators in the transmission
   * \return true if successful
   */
  static bool parseActuators(TiXmlElement * trans_it, std::vector<ActuatorInfo> & actuators);
}; // class


/**
 * \brief Parse transmission information from a URDF.
 * \param urdf A string containing the URDF xml
 * \return parsed transmission information
 * \throws std::runtime_error on malformed or empty xml
 */
TRANSMISSION_INTERFACE_PUBLIC
std::vector<TransmissionInfo> parse_transmissions_from_urdf(const std::string & urdf);

}  // namespace transmission_interface
