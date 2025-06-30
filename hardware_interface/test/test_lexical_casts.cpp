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
  ASSERT_TRUE(parse_bool("TRUE"));
  ASSERT_TRUE(parse_bool("TrUe"));

  // Any other value should return false
  ASSERT_FALSE(parse_bool("false"));
  ASSERT_FALSE(parse_bool("False"));
  ASSERT_FALSE(parse_bool("FALSE"));
  ASSERT_FALSE(parse_bool("fAlSe"));

  // Invalid inputs should throw an exception
  ASSERT_THROW(parse_bool("1"), std::invalid_argument);
  ASSERT_THROW(parse_bool("0"), std::invalid_argument);
  ASSERT_THROW(parse_bool("yes"), std::invalid_argument);
  ASSERT_THROW(parse_bool("no"), std::invalid_argument);
  ASSERT_THROW(parse_bool(""), std::invalid_argument);
  ASSERT_THROW(parse_bool("abc"), std::invalid_argument);
  ASSERT_THROW(parse_bool("1"), std::invalid_argument);
}

TEST(TestLexicalCasts, test_parse_string_array)
{
  using hardware_interface::parse_string_array;

  ASSERT_THROW(parse_string_array(""), std::invalid_argument);
  ASSERT_THROW(parse_string_array("abc"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("abc]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[[abc, def], hij]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[,]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc,]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[,abc]"), std::invalid_argument);
  ASSERT_THROW(parse_string_array("[abc,,def]"), std::invalid_argument);

  ASSERT_EQ(parse_string_array("[]"), std::vector<std::string>());
  ASSERT_EQ(parse_string_array("[ ]"), std::vector<std::string>());
  ASSERT_EQ(parse_string_array("[abc]"), std::vector<std::string>({"abc"}));
  ASSERT_EQ(parse_string_array("[abc,def]"), std::vector<std::string>({"abc", "def"}));
  ASSERT_EQ(parse_string_array("[abc, def]"), std::vector<std::string>({"abc", "def"}));
  ASSERT_EQ(parse_string_array("[ abc, def ]"), std::vector<std::string>({"abc", "def"}));
}

TEST(TestLexicalCasts, test_parse_double_array)
{
  using hardware_interface::parse_array;

  ASSERT_THROW(parse_array<double>(""), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("1.23"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[1.23"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("1.23]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[[1.23, 4.56], 7.89]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[ ,   ]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[1.23,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[,1.234]"), std::invalid_argument);
  ASSERT_THROW(parse_array<double>("[1.232,,1.324]"), std::invalid_argument);

  ASSERT_EQ(parse_array<double>("[]"), std::vector<double>());
  ASSERT_EQ(parse_array<double>("[ ]"), std::vector<double>());
  ASSERT_EQ(parse_array<double>("[1.23]"), std::vector<double>({1.23}));
  ASSERT_EQ(parse_array<double>("[-1.23]"), std::vector<double>({-1.23}));
  ASSERT_EQ(parse_array<double>("[1.23,4.56]"), std::vector<double>({1.23, 4.56}));
  ASSERT_EQ(parse_array<double>("[-1.23,-4.56]"), std::vector<double>({-1.23, -4.56}));
  ASSERT_EQ(parse_array<double>("[-1.23, 4.56]"), std::vector<double>({-1.23, 4.56}));
  ASSERT_EQ(parse_array<double>("[ -1.23, -124.56 ]"), std::vector<double>({-1.23, -124.56}));
  ASSERT_EQ(parse_array<double>("[ 1.23, 4 ]"), std::vector<double>({1.23, 4.0}));
  ASSERT_EQ(parse_array<double>("[ 1.23, 4.56, -7 ]"), std::vector<double>({1.23, 4.56, -7.0}));
  ASSERT_EQ(parse_array<double>("[ 1.23, 123, -7.89 ]"), std::vector<double>({1.23, 123.0, -7.89}));
  ASSERT_EQ(parse_array<double>("[ 1.23, 4.56, -7.89 ]"), std::vector<double>({1.23, 4.56, -7.89}));
}

TEST(TestLexicalCasts, test_parse_int_array)
{
  using hardware_interface::parse_array;

  ASSERT_THROW(parse_array<int>(""), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("123"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[232"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("123]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[[1.23, 4], 7]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[ ,   ]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[1,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[,1]"), std::invalid_argument);
  ASSERT_THROW(parse_array<int>("[1,,2]"), std::invalid_argument);

  ASSERT_EQ(parse_array<int>("[]"), std::vector<int>());
  ASSERT_EQ(parse_array<int>("[ ]"), std::vector<int>());
  ASSERT_EQ(parse_array<int>("[1]"), std::vector<int>({1}));
  ASSERT_EQ(parse_array<int>("[-1]"), std::vector<int>({-1}));
  ASSERT_EQ(parse_array<int>("[1,2]"), std::vector<int>({1, 2}));
  ASSERT_EQ(parse_array<int>("[-1,-2]"), std::vector<int>({-1, -2}));
  ASSERT_EQ(parse_array<int>("[ -1, -124 ]"), std::vector<int>({-1, -124}));
  ASSERT_EQ(parse_array<int>("[ -1, -124, +123 ]"), std::vector<int>({-1, -124, 123}));
}

TEST(TestLexicalCasts, test_parse_bool_array)
{
  using hardware_interface::parse_array;

  ASSERT_THROW(parse_array<bool>(""), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("true"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[true"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("true]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[[true, false], true]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[ ,   ]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[true,]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[,false]"), std::invalid_argument);
  ASSERT_THROW(parse_array<bool>("[true,,false]"), std::invalid_argument);

  ASSERT_EQ(parse_array<bool>("[]"), std::vector<bool>());
  ASSERT_EQ(parse_array<bool>("[ ]"), std::vector<bool>());
  ASSERT_EQ(parse_array<bool>("[true]"), std::vector<bool>({true}));
  ASSERT_EQ(parse_array<bool>("[false]"), std::vector<bool>({false}));
  ASSERT_EQ(parse_array<bool>("[true,false]"), std::vector<bool>({true, false}));
  ASSERT_EQ(parse_array<bool>("[false,true]"), std::vector<bool>({false, true}));
  ASSERT_EQ(parse_array<bool>("[ true, false ]"), std::vector<bool>({true, false}));
}
