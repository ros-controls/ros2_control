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

namespace ros2_control
{

/**
 * @brief Get the iterator to the item in the container.
 * @param container The container to search in.
 * @param item The item to search for.
 * @return Iterator to the item in the container.
 *
 * @note Only std::vector, std::map and std::unordered_map are supported.
 */
template <typename Container, typename T>
[[nodiscard]] auto get_item_iterator(const Container & container, const T & item)
{
  if constexpr (std::is_same_v<Container, std::vector<T>>)
  {
    return std::find(container.begin(), container.end(), item);
  }
  else if constexpr (
    std::is_same_v<Container, std::map<T, typename Container::mapped_type>> ||
    std::is_same_v<Container, std::unordered_map<T, typename Container::mapped_type>>)
  {
    return container.find(item);
  }
  else
  {
    using is_vector = std::is_same<Container, std::vector<T>>;
    using is_map = std::is_same<Container, std::map<T, typename Container::mapped_type>>;
    using is_unordered_map =
      std::is_same<Container, std::unordered_map<T, typename Container::mapped_type>>;
    // Handle unsupported container types
    static_assert(
      is_vector::value || is_map::value || is_unordered_map::value,
      "Only std::vector, std::map and std::unordered_map are supported.");
  }
}

/**
 * @brief Check if the item is in the container.
 * @param container The container to search in.
 * @param item The item to search for.
 * @return True if the item is in the container, false otherwise.
 */
template <typename Container, typename T>
[[nodiscard]] bool has_item(const Container & container, const T & item)
{
  return get_item_iterator(container, item) != container.end();
}

/**
 * @brief Add the item to the container if it is not already in it.
 * @param vector The container to add the item to.
 * @param item The item to add.
 */
template <typename T>
void add_item(std::vector<T> & vector, const T & item)
{
  if (!has_item(vector, item))
  {
    vector.push_back(item);
  }
}

/**
 * @brief Remove the item from the container if it is in it.
 * @param container The container to remove the item from.
 * @param item The item to remove.
 * @return True if the item was removed, false otherwise. If the item was not in the container, it
 * returns false.
 */
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

/**
 * @brief Check if the container has any of the items.
 * @param container The container to search in.
 * @param items The items to search for.
 * @return True if the container has any of the items, false otherwise.
 */
template <typename Container>
[[nodiscard]] bool has_any_item(
  const Container & container, const std::vector<typename Container::key_type> & items)
{
  return std::any_of(
    items.begin(), items.end(),
    [&container](const typename Container::key_type & item) { return has_item(container, item); });
}

/**
 * @brief Check if the container has any of the items.
 * @param container The container to search in.
 * @param items The items to search for.
 * @return True if the container has any of the items, false otherwise.
 */
template <typename T>
[[nodiscard]] bool has_any_item(const std::vector<T> & container, const std::vector<T> & items)
{
  return std::any_of(
    items.begin(), items.end(), [&container](const T & item) { return has_item(container, item); });
}

/**
 * @brief Check if the container has all of the items.
 * @param container The container to search in.
 * @param items The items to search for.
 * @return True if the container has all of the items, false otherwise.
 */
template <typename Container>
[[nodiscard]] bool has_all_items(
  const Container & container, const std::vector<typename Container::key_type> & items)
{
  return std::all_of(
    items.begin(), items.end(),
    [&container](const typename Container::key_type & item) { return has_item(container, item); });
}

/**
 * @brief Check if the container has all of the items.
 * @param container The container to search in.
 * @param items The items to search for.
 * @return True if the container has all of the items, false otherwise.
 */
template <typename T>
[[nodiscard]] bool has_all_items(const std::vector<T> & container, const std::vector<T> & items)
{
  return std::all_of(
    items.begin(), items.end(), [&container](const T & item) { return has_item(container, item); });
}

/**
 * @brief Check if the container has all unique items.
 * @param container The container to search in.
 * @return True if the container has all unique items, false otherwise.
 *
 * @note The container must be sortable.
 * @note The container must have the begin() and end() methods.
 */
template <typename Container>
[[nodiscard]] bool is_unique(Container container)
{
  std::sort(container.begin(), container.end());
  return std::adjacent_find(container.cbegin(), container.cend()) == container.cend();
}

}  // namespace ros2_control

#endif  // HARDWARE_INTERFACE__HELPERS_HPP_
