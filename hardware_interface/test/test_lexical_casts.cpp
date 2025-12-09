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
  ASSERT_THROW(stod("1.8e308"), std::invalid_argument);
  ASSERT_EQ(stod("1.2"), 1.2);
  ASSERT_EQ(stod("-1.2"), -1.2);
  ASSERT_EQ(stod("1.0"), 1.0);
}

TEST(TestLexicalCasts, test_stof)
{
  using hardware_interface::stof;

  ASSERT_THROW(stof(""), std::invalid_argument);
  ASSERT_THROW(stof("abc"), std::invalid_argument);
  ASSERT_THROW(stof("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stof("3.4e39"), std::invalid_argument);
  ASSERT_EQ(stof("1.2"), 1.2f);
  ASSERT_EQ(stof("-1.2"), -1.2f);
  ASSERT_EQ(stof("1.0"), 1.0f);
}

TEST(TestLexicalCasts, test_stoi8)
{
  using hardware_interface::stoi8;

  ASSERT_THROW(stoi8(""), std::invalid_argument);
  ASSERT_THROW(stoi8("abc"), std::invalid_argument);
  ASSERT_THROW(stoi8("1.2"), std::invalid_argument);
  ASSERT_THROW(stoi8("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoi8("999"), std::out_of_range);
  ASSERT_THROW(stoi8("-999"), std::out_of_range);
  ASSERT_THROW(stoi8("128"), std::out_of_range);   // INT8_MAX + 1
  ASSERT_THROW(stoi8("-129"), std::out_of_range);  // INT8_MIN - 1

  ASSERT_EQ(stoi8("0"), 0);
  ASSERT_EQ(stoi8("123"), 123);
  ASSERT_EQ(stoi8("-123"), -123);
  ASSERT_EQ(stoi8("127"), 127);    // INT8_MAX
  ASSERT_EQ(stoi8("-128"), -128);  // INT8_MIN
  ASSERT_EQ(stoi8("+123"), 123);
}

TEST(TestLexicalCasts, test_stoui8)
{
  using hardware_interface::stoui8;

  ASSERT_THROW(stoui8(""), std::invalid_argument);
  ASSERT_THROW(stoui8("abc"), std::invalid_argument);
  ASSERT_THROW(stoui8("1.2"), std::invalid_argument);
  ASSERT_THROW(stoui8("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoui8("-1"), std::out_of_range);    // Negative values not allowed
  ASSERT_THROW(stoui8("-123"), std::out_of_range);  // Negative values not allowed
  ASSERT_THROW(stoui8("256"), std::out_of_range);   // UINT8_MAX + 1
  ASSERT_THROW(stoui8("999"), std::out_of_range);

  ASSERT_EQ(stoui8("0"), 0u);
  ASSERT_EQ(stoui8("123"), 123u);
  ASSERT_EQ(stoui8("255"), 255u);  // UINT8_MAX
  ASSERT_EQ(stoui8("+123"), 123u);
}

TEST(TestLexicalCasts, test_stoi16)
{
  using hardware_interface::stoi16;

  ASSERT_THROW(stoi16(""), std::invalid_argument);
  ASSERT_THROW(stoi16("abc"), std::invalid_argument);
  ASSERT_THROW(stoi16("1.2"), std::invalid_argument);
  ASSERT_THROW(stoi16("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoi16("99999"), std::out_of_range);
  ASSERT_THROW(stoi16("-99999"), std::out_of_range);
  ASSERT_THROW(stoi16("32768"), std::out_of_range);   // INT16_MAX + 1
  ASSERT_THROW(stoi16("-32769"), std::out_of_range);  // INT16_MIN - 1

  ASSERT_EQ(stoi16("0"), 0);
  ASSERT_EQ(stoi16("123"), 123);
  ASSERT_EQ(stoi16("-123"), -123);
  ASSERT_EQ(stoi16("32767"), 32767);    // INT16_MAX
  ASSERT_EQ(stoi16("-32768"), -32768);  // INT16_MIN
  ASSERT_EQ(stoi16("+123"), 123);
}

TEST(TestLexicalCasts, test_stoui16)
{
  using hardware_interface::stoui16;

  ASSERT_THROW(stoui16(""), std::invalid_argument);
  ASSERT_THROW(stoui16("abc"), std::invalid_argument);
  ASSERT_THROW(stoui16("1.2"), std::invalid_argument);
  ASSERT_THROW(stoui16("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoui16("-1"), std::out_of_range);     // Negative values not allowed
  ASSERT_THROW(stoui16("-123"), std::out_of_range);   // Negative values not allowed
  ASSERT_THROW(stoui16("65536"), std::out_of_range);  // UINT16_MAX + 1
  ASSERT_THROW(stoui16("99999"), std::out_of_range);

  ASSERT_EQ(stoui16("0"), 0u);
  ASSERT_EQ(stoui16("123"), 123u);
  ASSERT_EQ(stoui16("65535"), 65535u);  // UINT16_MAX
  ASSERT_EQ(stoui16("+123"), 123u);
}

TEST(TestLexicalCasts, test_stoi32)
{
  using hardware_interface::stoi32;

  ASSERT_THROW(stoi32(""), std::invalid_argument);
  ASSERT_THROW(stoi32("abc"), std::invalid_argument);
  ASSERT_THROW(stoi32("1.2"), std::invalid_argument);
  ASSERT_THROW(stoi32("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoi32("9999999999"), std::out_of_range);
  ASSERT_THROW(stoi32("-9999999999"), std::out_of_range);
  ASSERT_THROW(stoi32("2147483648"), std::out_of_range);   // INT32_MAX + 1
  ASSERT_THROW(stoi32("-2147483649"), std::out_of_range);  // INT32_MIN - 1

  ASSERT_EQ(stoi32("0"), 0);
  ASSERT_EQ(stoi32("123"), 123);
  ASSERT_EQ(stoi32("-123"), -123);
  ASSERT_EQ(stoi32("2147483647"), 2147483647);    // INT32_MAX
  ASSERT_EQ(stoi32("-2147483648"), -2147483648);  // INT32_MIN
  ASSERT_EQ(stoi32("+123"), 123);
}

TEST(TestLexicalCasts, test_stoui32)
{
  using hardware_interface::stoui32;

  ASSERT_THROW(stoui32(""), std::invalid_argument);
  ASSERT_THROW(stoui32("abc"), std::invalid_argument);
  ASSERT_THROW(stoui32("1.2"), std::invalid_argument);
  ASSERT_THROW(stoui32("1.2.3"), std::invalid_argument);
  ASSERT_THROW(stoui32("-1"), std::out_of_range);          // Negative values not allowed
  ASSERT_THROW(stoui32("-123"), std::out_of_range);        // Negative values not allowed
  ASSERT_THROW(stoui32("4294967296"), std::out_of_range);  // UINT32_MAX + 1
  ASSERT_THROW(stoui32("9999999999"), std::out_of_range);

  ASSERT_EQ(stoui32("0"), 0u);
  ASSERT_EQ(stoui32("123"), 123u);
  ASSERT_EQ(stoui32("4294967295"), 4294967295u);  // UINT32_MAX
  ASSERT_EQ(stoui32("+123"), 123u);
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
