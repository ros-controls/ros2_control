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

#ifndef TEST_COMPONENTS_HPP_
#define TEST_COMPONENTS_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"

namespace test_components
{

template <typename C, typename T>
std::pair<bool, size_t> vector_contains(const std::vector<C> & vec, const T & element)
{
  auto it = std::find_if(
    vec.begin(), vec.end(), [element](const auto & state_interface)
    { return state_interface->get_name() == std::string(element); });

  return std::make_pair(it != vec.end(), std::distance(vec.begin(), it));
}

}  // namespace test_components
#endif  // TEST_COMPONENTS_HPP_
