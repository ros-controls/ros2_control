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
#include <vector>
#include "hardware_interface/robot_hardware.hpp"

namespace hw = hardware_interface;
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
    hw::return_type init() override
    {
      return hw::return_type::OK;
    }

    hw::return_type read() override
    {
      return hw::return_type::OK;
    }

    hw::return_type write() override
    {
      return hw::return_type::OK;
    }
  };

public:
  TestActuators()
  {
  }

protected:
  DummyRobotHardware robot_hw_;
};

TEST_F(TestActuators, no_actuators_registered_return_empty_on_all_fronts)
{
  EXPECT_THAT(robot_hw_.get_registered_actuators(), IsEmpty());
  EXPECT_THAT(robot_hw_.get_registered_actuator_names(), IsEmpty());

  hw::ActuatorHandle handle{"", ""};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_actuator_handle(handle));
}

TEST_F(TestActuators, can_register_actuator_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
}

TEST_F(TestActuators, can_not_double_register_actuator_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));

  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
}

TEST_F(TestActuators, can_not_register_with_empty_fields)
{
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_actuator("", ""));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_actuator(ACTUATOR_NAME, ""));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_actuator("", FOO_INTERFACE));
}

TEST_F(TestActuators, can_not_get_non_registered_actuators_or_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR2_NAME, BAR_INTERFACE));

  hw::ActuatorHandle handle1{ACTUATOR_NAME, BAR_INTERFACE};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_actuator_handle(handle1));
  hw::ActuatorHandle handle2{ACTUATOR2_NAME, FOO_INTERFACE};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_actuator_handle(handle2));
}

TEST_F(TestActuators, can_get_registered_actuators)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  EXPECT_THAT(robot_hw_.get_registered_actuator_names(), ElementsAre(ACTUATOR_NAME));
  const auto registered_actuators = robot_hw_.get_registered_actuators();
  EXPECT_THAT(registered_actuators, SizeIs(1));
  const auto & actuator_handle = registered_actuators[0];
  EXPECT_EQ(actuator_handle.get_name(), ACTUATOR_NAME);
  EXPECT_EQ(actuator_handle.get_interface_name(), FOO_INTERFACE);

  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
  EXPECT_THAT(robot_hw_.get_registered_actuator_names(), ElementsAre(ACTUATOR_NAME));

  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR2_NAME, FOO_INTERFACE));
  EXPECT_THAT(
    robot_hw_.get_registered_actuator_names(),
    UnorderedElementsAre(ACTUATOR_NAME, ACTUATOR2_NAME));
  EXPECT_THAT(
    robot_hw_.get_registered_actuator_interface_names(ACTUATOR_NAME),
    UnorderedElementsAre(FOO_INTERFACE, BAR_INTERFACE));
  EXPECT_THAT(
    robot_hw_.get_registered_actuator_interface_names(ACTUATOR2_NAME),
    UnorderedElementsAre(FOO_INTERFACE));

  std::vector<hw::ActuatorHandle> handles =
  {{ACTUATOR_NAME, FOO_INTERFACE}, {ACTUATOR_NAME, BAR_INTERFACE},
    {ACTUATOR2_NAME, FOO_INTERFACE}};

  for (auto & handle : handles) {
    EXPECT_EQ(hw::return_type::OK, robot_hw_.get_actuator_handle(handle));
  }
}

TEST_F(TestActuators, set_get_works_on_registered_actuators)
{
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR2_NAME, FOO_INTERFACE));
  auto actuator_handles = robot_hw_.get_registered_actuators();

  for (auto & handle : actuator_handles) {
    const auto new_value = handle.get_value() + 1.337;
    EXPECT_NO_THROW(handle.set_value(new_value));
    EXPECT_DOUBLE_EQ(handle.get_value(), new_value);
  }

  std::vector<hw::ActuatorHandle> handles =
  {{ACTUATOR_NAME, FOO_INTERFACE}, {ACTUATOR_NAME, BAR_INTERFACE},
    {ACTUATOR2_NAME, FOO_INTERFACE}};

  for (auto & handle : handles) {
    EXPECT_ANY_THROW(handle.get_value());
    EXPECT_ANY_THROW(handle.set_value(0.0));

    EXPECT_EQ(hw::return_type::OK, robot_hw_.get_actuator_handle(handle));
    const auto new_value = handle.get_value() + 1.337;
    EXPECT_NO_THROW(handle.set_value(new_value));
    EXPECT_DOUBLE_EQ(handle.get_value(), new_value);
  }
}

TEST_F(TestActuators, can_get_registered_actuators_of_interface)
{
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, FOO_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR_NAME, BAR_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_actuator(ACTUATOR2_NAME, FOO_INTERFACE));

  std::vector<hw::ActuatorHandle> handles1;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_actuator_handles(handles1, FOO_INTERFACE));
  ASSERT_EQ(handles1.size(), 2ul);
  for (const auto & handle : handles1) {
    ASSERT_EQ(handle.get_interface_name(), FOO_INTERFACE);
  }

  std::vector<hw::ActuatorHandle> handles2;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_actuator_handles(handles2, BAR_INTERFACE));
  ASSERT_EQ(handles2.size(), 1ul);
  for (const auto & handle : handles2) {
    ASSERT_EQ(handle.get_interface_name(), BAR_INTERFACE);
  }

  std::vector<hw::ActuatorHandle> handles3;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_actuator_handles(handles3, "NoInterface"));
  ASSERT_TRUE(handles3.empty());
}
