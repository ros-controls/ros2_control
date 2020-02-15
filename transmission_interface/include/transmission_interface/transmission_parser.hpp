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

#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_

#include <tinyxml2.h>

#include <string>
#include <vector>

#include "transmission_interface/transmission_info.hpp"
#include "transmission_interface/visibility_control.h"

namespace transmission_interface
{

class TRANSMISSION_INTERFACE_PUBLIC_TYPE TransmissionParser
{
public:
  explicit TransmissionParser(const std::string & urdf);

  bool parse_transmission_info(std::vector<TransmissionInfo> & transmissions) const;

  std::string get_error_msg() const;

protected:
  void set_error_msg(std::string error_msg) const;

  void reset_error_msg() const;

  mutable std::string error_msg_;

  std::string urdf_;
  tinyxml2::XMLDocument doc_;
};

}  // namespace transmission_interface
#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_PARSER_HPP_
