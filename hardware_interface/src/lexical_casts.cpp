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
#include <regex>
#include <stdexcept>
#include <type_traits>
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

template <typename T>
std::vector<T> parse_array(const std::string & array_string)
{
  // Use regex to check for a flat array: starts with [, ends with ], no nested brackets
  const std::regex array_regex(R"(^\[\s*([^\[\]]*\s*(,\s*[^\[\]]+\s*)*)?\]$)");
  if (!std::regex_match(array_string, array_regex))
  {
    throw std::invalid_argument("String must be a flat array: starts with '[' and ends with ']', no nested arrays");
  }

  // Use regex for the expression that either empty or contains only spaces
  const std::regex empty_or_spaces_regex(R"(^\[\s*\]$)");
  if (std::regex_match(array_string, empty_or_spaces_regex))
  {
    return {};  // Return empty array if input is "[]"
  }

  // Use regex to find cases of comma-separated but only whitespaces or no spaces between them like "[,]" "[a,b,,c]"
  const std::regex comma_separated_regex(R"(^\[\s*([^,\s]+(\s*,\s*[^,\s]+)*)?\s*\]$)");
  if (!std::regex_match(array_string, comma_separated_regex))
  {
    throw std::invalid_argument("String must be a flat array with comma-separated values and no spaces between them");
  }

  std::vector<T> result = {};
  if (array_string == "[]")
  {
    return result;  // Return empty array if input is "[]"
  }

  //regex for comma separated values and no spaces between them or just content like "[a,b,c]" or "[a]" or "[a, b, c]"
  // The regex captures values between commas, allowing for optional spaces around them
  // It captures the first group of non-whitespace characters that are not commas
  // and allows for multiple such groups separated by commas.
  // Example matches: "a", "b", "c", "a, b", "a, b, c"
  // It does not allow nested arrays or empty values.
  const std::regex value_regex(R"(\s*([^,\s]+)\s*)");
  auto it = std::sregex_iterator(array_string.begin(), array_string.end(), value_regex);
  auto end = std::sregex_iterator();

  for (; it != end; ++it)
  {
    const std::string value_str = it->str(1);  // Get the first capturing group
    if constexpr (std::is_same_v<T, std::string>)
    {
      result.push_back(value_str);
    }
    else if constexpr (std::is_floating_point_v<T> || std::is_integral_v<T>)
    {
      if (const auto value = impl::stod(value_str))
      {
        result.push_back(static_cast<T>(*value));
      }
      else
      {
        throw std::invalid_argument("Failed converting string to floating point or integer: " + value_str);
      }
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
      result.push_back(parse_bool(value_str));
    }
    else
    {
      throw std::invalid_argument("Unsupported type for parsing: " + std::string(typeid(T).name()));
    }
  }
  return result;
}

std::vector<std::string> parse_string_array(const std::string & string_array_string)
{
  return parse_array<std::string>(string_array_string);
}

}  // namespace hardware_interface
