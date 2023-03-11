// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "transmission_interface/accessor.hpp"
#include "transmission_interface/handle.hpp"

using hardware_interface::HW_IF_POSITION;
using transmission_interface::JointHandle;

TEST(UtilsTest, AccessorTest)
{
  const std::string NAME = "joint";
  double joint_value = 0.0;
  const JointHandle joint_handle(NAME, HW_IF_POSITION, &joint_value);
  const std::vector<JointHandle> joint_handles = {joint_handle};

  ASSERT_EQ(transmission_interface::get_names(joint_handles), std::vector<std::string>{NAME});
  ASSERT_EQ(
    transmission_interface::get_ordered_handles(joint_handles, {NAME}, HW_IF_POSITION),
    joint_handles);
}
