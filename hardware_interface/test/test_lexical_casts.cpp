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

#include <stdexcept>

#include "gtest/gtest.h"

#include "hardware_interface/lexical_casts.hpp"

TEST(TestLexicalCasts, test_stod)
{
  using hardware_interface::stod;

  ASSERT_THROW(stod(""), std::invalid_argument);
  ASSERT_THROW(stod("abc"), std::invalid_argument);
  ASSERT_THROW(stod("1.2.3"), std::invalid_argument);
  ASSERT_EQ(stod("1.2"), 1.2);
  ASSERT_EQ(stod("-1.2"), -1.2);
  ASSERT_EQ(stod("1.0"), 1.0);
}

TEST(TestLexicalCasts, test_parse_bool)
{
  using hardware_interface::parse_bool;

  ASSERT_TRUE(parse_bool("true"));
  ASSERT_TRUE(parse_bool("True"));

  // Any other value should return false
  ASSERT_FALSE(parse_bool("false"));
  ASSERT_FALSE(parse_bool("False"));
  ASSERT_FALSE(parse_bool(""));
  ASSERT_FALSE(parse_bool("abc"));
  ASSERT_FALSE(parse_bool("1"));
}

TEST(TestLexicalCasts, test_parse_string_array)
{
  using hardware_interface::parse_string_array;

  ASSERT_THROW(parse_string_array(""), std::invalid_argument);
  ASSERT_THROW(parse_string_array("abc"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("abc]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[[abc, def], hij]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[ ]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[,]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc,]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[,abc]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc,,def]"), std::invalid_argument);

  ASSERT_EQ(parse_string_array("[]"), std::vector<std::string>());
  ASSERT_EQ(parse_string_array("[abc]"), std::vector<std::string>({"abc"}));
  ASSERT_EQ(parse_string_array("[abc,def]"), std::vector<std::string>({"abc", "def"}));
  ASSERT_EQ(parse_string_array("[abc, def]"), std::vector<std::string>({"abc", "def"}));
  ASSERT_EQ(parse_string_array("[ abc, def ]"), std::vector<std::string>({"abc", "def"}));
}
