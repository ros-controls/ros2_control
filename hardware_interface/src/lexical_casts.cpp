// Copyright 2024 ros2_control Development Team
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

<<<<<<< HEAD
=======
#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

>>>>>>> c178e16 (Add missing `charconv` header (#3234))
#include "hardware_interface/lexical_casts.hpp"

namespace hardware_interface
{
double stod(const std::string & s)
{
  // convert from string using no locale
  std::istringstream stream(s);
  stream.imbue(std::locale::classic());
  double result;
  stream >> result;
  if (stream.fail() || !stream.eof())
  {
    throw std::invalid_argument("Failed converting string to real number");
  }
  return result;
}

bool parse_bool(const std::string & bool_string)
{
  return bool_string == "true" || bool_string == "True";
}
}  // namespace hardware_interface
