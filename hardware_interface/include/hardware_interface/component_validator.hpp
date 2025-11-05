// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__COMPONENT_VALIDATOR_HPP_
#define HARDWARE_INTERFACE__COMPONENT_VALIDATOR_HPP_

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hardware_interface
{

/// Validate URDF string against an XML Schema Definition (XSD) file.
/**
 * \param[in] urdf string with robot's URDF
 * \param[in] xsd_file_path path to the XSD file
 * \return true if the URDF is valid according to the XSD, false otherwise
 */
bool validate_urdf_with_xsd(const std::string & urdf, const std::string & xsd_file_path);

/// Validate URDF file path against an XML Schema Definition (XSD) file.
/**
 * \param[in] urdf_file_path path to URDF file
 * \param[in] xsd_file_path path to the XSD file
 * \return true if URDF is valid according to the XSD, false otherwise
 *
 */
bool validate_urdf_file_path_with_xsd(
  const std::string & urdf_file_path, std::string & xsd_file_path);

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__COMPONENT_VALIDATOR_HPP_
