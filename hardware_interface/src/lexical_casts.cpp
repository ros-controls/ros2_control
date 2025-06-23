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

#include <locale>
#include <optional>
#include <sstream>
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

bool parse_bool(const std::string & bool_string)
{
  return bool_string == "true" || bool_string == "True";
}

std::vector<std::string> parse_string_array(const std::string & string_array_string)
{
  // Check string starts with '[' and ends with ']'
  if (
    string_array_string.empty() || string_array_string.front() != '[' ||
    string_array_string.back() != ']')
  {
    throw std::invalid_argument("String must start with '[' and end with ']'");
  }

  // Check there are no "sub arrays"
  if (
    string_array_string.find("[") != 0u ||
    string_array_string.find("]") != string_array_string.size() - 1u)
  {
    throw std::invalid_argument("String contains nested arrays");
  }

  // Check for empty array
  if (string_array_string == "[]")
  {
    return {};
  }

  std::vector<std::string> result;
  std::string current_string;
  for (char c : string_array_string)
  {
    if (c == ',' || c == ']')
    {
      if (!current_string.empty())
      {
        result.push_back(current_string);
        current_string.clear();
      }
      else
      {
        throw std::invalid_argument("Empty string found in array");
      }
    }
    else if (c == '[' || c == ' ')
    {
      // Ignore opening brackets and spaces
    }
    else
    {
      current_string += c;  // Add character to current string
    }
  }

  return result;
}

}  // namespace hardware_interface
