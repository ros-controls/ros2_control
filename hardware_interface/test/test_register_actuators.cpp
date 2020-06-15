// Copyright 2020 ros2_control development team
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
#include <string>
#include "hardware_interface/robot_hardware.hpp"

namespace hw = hardware_interface;
using hardware_interface::HW_RET_OK;
using hardware_interface::HW_RET_ERROR;
using testing::SizeIs;
using testing::IsEmpty;
using testing::Each;
using testing::NotNull;
using testing::UnorderedElementsAre;
using testing::ElementsAre;

namespace
{
constexpr auto ACTUATOR_NAME = "actuator_1";
constexpr auto ACTUATOR2_NAME = "neck_tilt_motor";
constexpr auto FOO_INTERFACE = "FooInterface";
constexpr auto BAR_INTERFACE = "BarInterface";
}  // namespace

class TestActuators : public testing::Test
{
  class DummyRobotHardware : public hw::RobotHardware
  {
    hw::hardware_interface_ret_t init() override
    {
      return HW_RET_OK;
    }

    hw::hardware_interface_ret_t read() override
    {
      return HW_RET_OK;
    }

    hw::hardware_interface_ret_t write() override
    {
      return HW_RET_OK;
    }
  };

public:
  TestActuators()
  {
  }

protected:
  DummyRobotHardware robot_hw;
};

TEST_F(TestActuators, no_actuators_registered_return_empty_on_all_fronts)
{
  EXPECT_THAT(robot_hw.get_registered_actuators(), IsEmpty());
  EXPECT_THAT(robot_hw.get_registered_actuator_names(), IsEmpty());

  hw::ActuatorHandle handle{"", ""};
  EXPECT_EQ(HW_RET_ERROR, robot_hw.get_actuator_handle(handle));
}

TEST_F(TestActuators, can_register_actuator_interfaces)
{
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
}

TEST_F(TestActuators, can_not_double_register_actuator_interfaces)
{
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));

  EXPECT_EQ(HW_RET_ERROR, robot_hw.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(HW_RET_ERROR, robot_hw.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
}

TEST_F(TestActuators, can_not_register_with_empty_fields)
{
  EXPECT_EQ(HW_RET_ERROR, robot_hw.register_actuator("", ""));
  EXPECT_EQ(HW_RET_ERROR, robot_hw.register_actuator(ACTUATOR_NAME, ""));
  EXPECT_EQ(HW_RET_ERROR, robot_hw.register_actuator("", FOO_INTERFACE));
}

TEST_F(TestActuators, can_not_get_non_registered_actuators_or_interfaces)
{
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR2_NAME, BAR_INTERFACE));

  hw::ActuatorHandle handle1{ACTUATOR_NAME, BAR_INTERFACE};
  EXPECT_EQ(HW_RET_ERROR, robot_hw.get_actuator_handle(handle1));
  hw::ActuatorHandle handle2{ACTUATOR2_NAME, FOO_INTERFACE};
  EXPECT_EQ(HW_RET_ERROR, robot_hw.get_actuator_handle(handle2));
}

TEST_F(TestActuators, can_get_registered_actuators)
{
  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_THAT(robot_hw.get_registered_actuator_names(), ElementsAre(ACTUATOR_NAME));
  const auto registered_actuators = robot_hw.get_registered_actuators();
  EXPECT_THAT(registered_actuators, SizeIs(1));
  const auto & actuator_handle = registered_actuators[0];
  EXPECT_EQ(actuator_handle.get_name(), ACTUATOR_NAME);
  EXPECT_EQ(actuator_handle.get_interface_name(), FOO_INTERFACE);

  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
  EXPECT_THAT(robot_hw.get_registered_actuator_names(), ElementsAre(ACTUATOR_NAME));

  EXPECT_EQ(HW_RET_OK, robot_hw.register_actuator(ACTUATOR2_NAME, FOO_INTERFACE));
  EXPECT_THAT(
    robot_hw.get_registered_actuator_names(),
    UnorderedElementsAre(ACTUATOR_NAME, ACTUATOR2_NAME));
}
