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

#include <gmock/gmock.h>
#include "hardware_interface/helpers.hpp"

TEST(TestHelper, test_helper_methods)
{
  std::vector<std::string> vec({"aa", "bb", "cc"});
  std::map<std::string, int> map{{"aa", 1}, {"bb", 2}, {"cc", 3}};
  std::unordered_map<std::string, int> umap{{"aa", 1}, {"bb", 2}, {"cc", 3}};

  for (std::string var : {"aa", "bb", "cc"})
  {
    ASSERT_TRUE(ros2_control::has_item(vec, var));
    ASSERT_TRUE(ros2_control::has_item(map, var));
    ASSERT_TRUE(ros2_control::has_item(umap, var));

    ASSERT_TRUE(ros2_control::has_any_item(vec, {var}));
    ASSERT_TRUE(ros2_control::has_any_item(map, {var}));
    ASSERT_TRUE(ros2_control::has_any_item(umap, {var}));

    ASSERT_NE(ros2_control::get_item_iterator(vec, var), vec.end());
    ASSERT_NE(ros2_control::get_item_iterator(map, var), map.end());
    ASSERT_NE(ros2_control::get_item_iterator(umap, var), umap.end());
  }

  ASSERT_TRUE(ros2_control::has_all_items(vec, {"aa", "bb", "cc"}));
  ASSERT_TRUE(ros2_control::has_all_items(map, {"aa", "bb", "cc"}));
  ASSERT_TRUE(ros2_control::has_all_items(umap, {"aa", "bb", "cc"}));

  const std::string dd_str = "dd";
  ASSERT_FALSE(ros2_control::has_item(vec, dd_str));
  ASSERT_EQ(vec.size(), 3u);
  ros2_control::add_item(vec, dd_str);
  ASSERT_TRUE(ros2_control::has_item(vec, dd_str));
  ASSERT_EQ(vec.size(), 4u);
  ASSERT_TRUE(ros2_control::remove_item(vec, dd_str));
  ASSERT_FALSE(ros2_control::has_item(vec, dd_str));
  ASSERT_EQ(vec.size(), 3u);
  ASSERT_FALSE(ros2_control::remove_item(vec, std::string("ee")));
  ASSERT_EQ(vec.size(), 3u);

  ASSERT_TRUE(ros2_control::is_unique(vec));
  ASSERT_FALSE(ros2_control::is_unique(std::vector<std::string>({"aa", "bb", "cc", "aa"})));
}
