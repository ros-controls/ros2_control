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

#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace hardware_interface
{

/** \brief Helper function to convert a std::string to double in a locale-independent way.
 * \throws std::invalid_argument if not a valid number or exceeds limits
 * from
 https://github.com/ros-planning/srdfdom/blob/ad17b8d25812f752c397a6011cec64aeff090c46/src/model.cpp#L53
*/
double stod(const std::string & s);

/** \brief Helper function to convert a std::string to float in a locale-independent way.
 * \throws std::invalid_argument if not a valid number or exceeds limits
 */
float stof(const std::string & s);

/** \brief Overflow-safe conversion from string to int32_t.
 * \throws std::out_of_range if the converted value would fall out of the range of int32_t
 * \throws std::invalid_argument if no conversion could be performed
 */
template <typename T>
T stoi_generic(const std::string & s)
{
  static_assert(std::is_integral_v<T> && std::is_signed_v<T>, "T must be a signed integral type");

  size_t pos;
  const auto v = std::stol(s, &pos);  // stol returns long
  if (pos != s.length())
  {
    throw std::invalid_argument("Invalid characters in string");
  }
  if (v < std::numeric_limits<T>::min() || v > std::numeric_limits<T>::max())
  {
    throw std::out_of_range("value not in range of target type");
  }

  return static_cast<T>(v);
}

// Explicit instantiations for common signed integer types
inline int8_t stoi8(const std::string & s) { return stoi_generic<int8_t>(s); }
inline int16_t stoi16(const std::string & s) { return stoi_generic<int16_t>(s); }
inline int32_t stoi32(const std::string & s) { return stoi_generic<int32_t>(s); }

/** \brief Overflow-safe conversion from string to uint32_t.
 * \throws std::out_of_range if the converted value would fall out of the range of int32_t
 * \throws std::invalid_argument if no conversion could be performed
 */
template <typename T>
T stoui_generic(const std::string & s)
{
  static_assert(
    std::is_integral_v<T> && std::is_unsigned_v<T>, "T must be an unsigned integral type");

  size_t pos;
  const auto v = std::stoul(s, &pos);  // stoul returns unsigned long
  if (pos != s.length())
  {
    throw std::invalid_argument("Invalid characters in string");
  }
  if (v > std::numeric_limits<T>::max())
  {
    throw std::out_of_range("value not in range of target type");
  }

  return static_cast<T>(v);
}

// Explicit instantiations for common unsigned integer types
inline uint8_t stoui8(const std::string & s) { return stoui_generic<uint8_t>(s); }
inline uint16_t stoui16(const std::string & s) { return stoui_generic<uint16_t>(s); }
inline uint32_t stoui32(const std::string & s) { return stoui_generic<uint32_t>(s); }

/**
 * \brief Convert a string to lower case.
 * \param string The input string.
 * \return The lower case version of the input string.
 */
std::string to_lower_case(const std::string & string);

/**
 * \brief Parse a boolean value from a string.
 * \param bool_string The input string, can be "true", "false" (case insensitive)
 * \return The parsed boolean value.
 * \throws std::invalid_argument if the string is not a valid boolean representation.
 */
bool parse_bool(const std::string & bool_string);

template <typename T>
std::vector<T> parse_array(const std::string & array_string)
{
  // Use regex to check for a flat array: starts with [, ends with ], no nested brackets
  const std::regex array_regex(R"(^\[\s*([^\[\]]*\s*(,\s*[^\[\]]+\s*)*)?\]$)");
  if (!std::regex_match(array_string, array_regex))
  {
    throw std::invalid_argument(
      "String must be a flat array: starts with '[' and ends with ']', no nested arrays");
  }

  // Use regex for the expression that either empty or contains only spaces
  const std::regex empty_or_spaces_regex(R"(^\[\s*\]$)");
  if (std::regex_match(array_string, empty_or_spaces_regex))
  {
    return {};  // Return empty array if input is "[]"
  }

  // Use regex to find cases of comma-separated but only whitespaces or no spaces between them like
  // "[,]" "[a,b,,c]"
  const std::regex comma_separated_regex(R"(^\[\s*([^,\s]+(\s*,\s*[^,\s]+)*)?\s*\]$)");
  if (!std::regex_match(array_string, comma_separated_regex))
  {
    throw std::invalid_argument(
      "String must be a flat array with comma-separated values and no spaces between them");
  }

  std::vector<T> result = {};
  if (array_string == "[]")
  {
    return result;  // Return empty array if input is "[]"
  }

  // regex for comma separated values and no spaces between them or just content like "[a,b,c]" or
  // "[a]" or "[a, b, c]"
  const std::regex value_regex(R"([^\s,\[\]]+)");
  auto begin = std::sregex_iterator(array_string.begin(), array_string.end(), value_regex);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it)
  {
    const std::string value_str = it->str();  // Get the first capturing group
    if constexpr (std::is_same_v<T, std::string>)
    {
      result.push_back(value_str);
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
      result.push_back(parse_bool(value_str));
    }
    else if constexpr (std::is_floating_point_v<T> || std::is_integral_v<T>)
    {
      try
      {
        const T value = static_cast<T>(hardware_interface::stod(value_str));
        result.push_back(value);
      }
      catch (const std::exception &)
      {
        throw std::invalid_argument(
          "Failed converting string to floating point or integer: " + value_str);
      }
    }
    else
    {
      throw std::invalid_argument("Unsupported type for parsing: " + std::string(typeid(T).name()));
    }
  }
  return result;
}

std::vector<std::string> parse_string_array(const std::string & string_array_string);

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__LEXICAL_CASTS_HPP_
