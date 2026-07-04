// Copyright 2020 PAL Robotics S.L.
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

#ifndef TRANSMISSION_INTERFACE__ACCESSOR_HPP_
#define TRANSMISSION_INTERFACE__ACCESSOR_HPP_

#include <algorithm>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace transmission_interface
{
template <typename T>
std::string to_string(const std::vector<T> & list)
{
  std::stringstream ss;
  ss << "[";
  for (const auto & elem : list)
  {
    ss << elem << ", ";
  }

  if (!list.empty())
  {
    ss.seekp(-2, std::ios_base::end);  // remove last ", "
  }
  ss << "]";
  return ss.str();
}

template <class T>
std::vector<std::string> get_names(const std::vector<T> & handles)
{
  std::set<std::string> names;
  std::transform(
    handles.cbegin(), handles.cend(), std::inserter(names, names.end()),
    [](const auto & handle) { return handle.get_prefix_name(); });
  return std::vector<std::string>(names.begin(), names.end());
}

template <typename T>
std::vector<T> get_ordered_handles(
  const std::vector<T> & unordered_handles, const std::vector<std::string> & names,
  const std::string & interface_type)
{
  std::vector<T> result;
  for (const auto & name : names)
  {
    std::copy_if(
      unordered_handles.cbegin(), unordered_handles.cend(), std::back_inserter(result),
      [&](const auto & handle)
      {
        return (handle.get_prefix_name() == name) &&
               (handle.get_interface_name() == interface_type) && handle;
      });
  }
  return result;
}

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__ACCESSOR_HPP_
