// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <vector>

#include "hardware_interface/macros.hpp"

class TestMacros : public ::testing::Test
{
protected:
  static void SetUpTestCase() {}
};

class A
{
};

TEST_F(TestMacros, throw_on_null)
{
  int * i_ptr = nullptr;
  // cppcheck-suppress unknownMacro
  EXPECT_ANY_THROW(THROW_ON_NULLPTR(i_ptr));

  A * a_ptr = nullptr;
  EXPECT_ANY_THROW(THROW_ON_NULLPTR(a_ptr));

  std::vector<int *> vec_ptr(7);
  for (auto ptr : vec_ptr)
  {
    EXPECT_ANY_THROW(THROW_ON_NULLPTR(ptr));
  }

  int ** i_ptr_ptr = &i_ptr;
  EXPECT_ANY_THROW(THROW_ON_NULLPTR(*i_ptr_ptr));

  /*
  // undefined behavior
  int * i_ptr;
  THROW_ON_NULLPTR(i_ptr);
  */
}

TEST_F(TestMacros, throw_on_not_null)
{
  int * i_ptr = new int();
  EXPECT_ANY_THROW(THROW_ON_NOT_NULLPTR(i_ptr));

  A * a_ptr = new A();
  EXPECT_ANY_THROW(THROW_ON_NOT_NULLPTR(a_ptr));

  std::vector<int *> vec_ptr;
  for (auto i : {0, 1, 2})
  {
    vec_ptr.push_back(i_ptr);
    EXPECT_ANY_THROW(THROW_ON_NOT_NULLPTR(vec_ptr[i]));
  }

  int ** i_ptr_ptr = &i_ptr;
  EXPECT_ANY_THROW(THROW_ON_NOT_NULLPTR(*i_ptr_ptr));

  delete a_ptr;
}
