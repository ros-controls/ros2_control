// Copyright 2023 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__LEXICAL_CASTS_HPP_
#define HARDWARE_INTERFACE__LEXICAL_CASTS_HPP_

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace hardware_interface
{

/** \brief Helper function to convert a std::string to double in a locale-independent way.
 \throws std::invalid_argument if not a valid number
 * from
 https://github.com/ros-planning/srdfdom/blob/ad17b8d25812f752c397a6011cec64aeff090c46/src/model.cpp#L53
*/
double stod(const std::string & s);

bool parse_bool(const std::string & bool_string);

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__LEXICAL_CASTS_HPP_
