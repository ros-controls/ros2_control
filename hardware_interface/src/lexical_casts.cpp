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

#include <algorithm>
#include <cctype>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"

namespace hardware_interface
{
namespace impl
{
std::optional<double> stod(const std::string & s)
{
#if __cplusplus < 202002L
  // convert from string using no locale
  // Impl with std::istringstream
  std::istringstream stream(s);
  stream.imbue(std::locale::classic());
  double result;
  stream >> result;
  if (stream.fail() || !stream.eof())
  {
    return std::nullopt;
  }
  return result;
#else
  // Impl with std::from_chars
  double result_value;
  const auto parse_result = std::from_chars(s.data(), s.data() + s.size(), result_value);
  if (parse_result.ec == std::errc())
  {
    return result_value;
  }
  return std::nullopt;
#endif
}
}  // namespace impl

double stod(const std::string & s)
{
  if (const auto result = impl::stod(s))
  {
    return *result;
  }
  throw std::invalid_argument("Failed converting string to real number");
}

std::string to_lower_case(const std::string & string)
{
  std::string lower_case_string = string;
  std::transform(
    lower_case_string.begin(), lower_case_string.end(), lower_case_string.begin(),
    [](unsigned char c) { return std::tolower(c); });
  return lower_case_string;
}

bool parse_bool(const std::string & bool_string)
{
  // Copy input to temp and make lowercase
  std::string temp = to_lower_case(bool_string);

  if (temp == "true")
  {
    return true;
  }
  if (temp == "false")
  {
    return false;
  }
  // If input is not "true" or "false" (any casing), throw or handle as error
  throw std::invalid_argument(
    "Input string : '" + bool_string +
    "' is not a valid boolean value. Expected 'true' or 'false'.");
}

std::vector<std::string> parse_string_array(const std::string & string_array_string)
{
  return parse_array<std::string>(string_array_string);
}

}  // namespace hardware_interface
