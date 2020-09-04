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

#include <gmock/gmock.h>
#include "hardware_interface/joint_handle.hpp"

using hardware_interface::JointHandle;

namespace
{
constexpr auto JOINT_NAME = "joint_1";
constexpr auto FOO_INTERFACE = "FooInterface";
}  // namespace

TEST(TestJointHandle, name_getters_work)
{
  JointHandle handle{JOINT_NAME, FOO_INTERFACE};
  EXPECT_EQ(handle.get_name(), JOINT_NAME);
  EXPECT_EQ(handle.get_interface_name(), FOO_INTERFACE);
}

TEST(TestJointHandle, value_methods_throw_for_nullptr)
{
  JointHandle handle{JOINT_NAME, FOO_INTERFACE};
  EXPECT_ANY_THROW(handle.get_value());
  EXPECT_ANY_THROW(handle.set_value(0.0));
}

TEST(TestJointHandle, value_methods_work_on_non_nullptr)
{
  double value = 1.337;
  JointHandle handle{JOINT_NAME, FOO_INTERFACE, &value};
  EXPECT_DOUBLE_EQ(handle.get_value(), value);
  EXPECT_NO_THROW(handle.set_value(0.0));
  EXPECT_DOUBLE_EQ(handle.get_value(), 0.0);
}

TEST(TestJointHandle, with_value_ptr_initializes_new_handle_correctly)
{
  double value = 1.337;
  JointHandle handle{JOINT_NAME, FOO_INTERFACE};
  auto new_handle = handle.with_value_ptr(&value);
  EXPECT_ANY_THROW(handle.get_value());
  EXPECT_DOUBLE_EQ(new_handle.get_value(), value);
}
