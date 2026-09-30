// Copyright 2026 ros2_control Development Team
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

// All transmission headers must be usable together in a single translation unit.
#include "gmock/gmock.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/differential_transmission.hpp"
#include "transmission_interface/four_bar_linkage_transmission.hpp"
#include "transmission_interface/simple_transmission.hpp"

TEST(IncludeAllTransmissions, AbsolutePositionInterfaceNameIsDefinedByHardwareInterface)
{
  EXPECT_STREQ(hardware_interface::HW_IF_ABSOLUTE_POSITION, "absolute_position");
}

TEST(IncludeAllTransmissions, TransmissionInterfaceNameRefersToHardwareInterfaceConstant)
{
  EXPECT_EQ(
    &transmission_interface::HW_IF_ABSOLUTE_POSITION, &hardware_interface::HW_IF_ABSOLUTE_POSITION);
}
