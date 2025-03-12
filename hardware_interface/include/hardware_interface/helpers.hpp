// Copyright 2025 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__HELPERS_HPP_
#define HARDWARE_INTERFACE__HELPERS_HPP_

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace helpers
{

template <typename Container, typename T>
[[nodiscard]] auto get_item_iterator(const Container & container, const T & item)
{
  if constexpr (std::is_same_v<Container, std::vector<T>>)
  {
    return std::find(container.begin(), container.end(), item);
  }
  else if constexpr (
    std::is_same_v<Container, std::map<typename Container::key_type, T>> ||
    std::is_same_v<Container, std::unordered_map<typename Container::key_type, T>>)
  {
    return container.find(item);
  }
  else
  {
    using is_vector = std::is_same<Container, std::vector<T>>;
    using is_map = std::is_same<Container, std::map<typename Container::key_type, T>>;
    using is_unordered_map =
      std::is_same<Container, std::unordered_map<typename Container::key_type, T>>;
    // Handle unsupported container types
    static_assert(
      is_vector::value || is_map::value || is_unordered_map::value,
      "Only std::vector, std::map and std::unordered_map are supported.");
  }
}

template <typename Container>
[[nodiscard]] bool has_item(const Container & container, typename Container::const_reference item)
{
  return get_item_iterator(container, item) != container.end();
}

template <typename T>
void add_item(std::vector<T> & vector, const T & item)
{
  if (!has_item(vector, item))
  {
    vector.push_back(item);
  }
}

template <typename Container>
[[nodiscard]] bool remove_item(Container & container, typename Container::const_reference item)
{
  auto it = get_item_iterator(container, item);
  if (it != container.end())
  {
    container.erase(it);
    return true;
  }
  return false;
}

template <typename Container>
[[nodiscard]] bool has_any_item(
  const Container & container, const std::vector<typename Container::key_type> & items)
{
  return std::any_of(
    items.begin(), items.end(),
    [&container](const typename Container::key_type & item) { return has_item(container, item); });
}

template <typename Container>
[[nodiscard]] bool has_all_items(
  const Container & container, const std::vector<typename Container::key_type> & items)
{
  return std::all_of(
    items.begin(), items.end(),
    [&container](const typename Container::key_type & item) { return has_item(container, item); });
}

template <typename Collection>
[[nodiscard]] bool is_unique(Collection collection)
{
  std::sort(collection.begin(), collection.end());
  return std::adjacent_find(collection.cbegin(), collection.cend()) == collection.cend();
}

}  // namespace helpers

#endif  // HARDWARE_INTERFACE__HELPERS_HPP_
