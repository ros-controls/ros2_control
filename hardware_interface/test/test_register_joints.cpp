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
constexpr auto JOINT_NAME = "joints_1";
constexpr auto JOINT2_NAME = "neck_tilt_motor";
constexpr auto FOO_INTERFACE = "FooInterface";
constexpr auto BAR_INTERFACE = "BarInterface";
}  // namespace

class TestJoints : public testing::Test
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
  TestJoints()
  {
  }

protected:
  DummyRobotHardware robot_hw_;
};

TEST_F(TestJoints, no_jointss_registered_return_empty_on_all_fronts)
{
  EXPECT_THAT(robot_hw_.get_registered_joints(), IsEmpty());
  EXPECT_THAT(robot_hw_.get_registered_joint_names(), IsEmpty());

  hw::JointHandle handle{"", ""};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_joint_handle(handle));
}

TEST_F(TestJoints, can_register_joint_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));
}

TEST_F(TestJoints, can_not_double_register_joint_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));

  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));
}

TEST_F(TestJoints, can_not_register_with_empty_fields)
{
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_joint("", ""));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_joint(JOINT_NAME, ""));
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.register_joint("", FOO_INTERFACE));
}

TEST_F(TestJoints, can_not_get_non_registered_jointss_or_interfaces)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT2_NAME, BAR_INTERFACE));

  hw::JointHandle handle1{JOINT_NAME, BAR_INTERFACE};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_joint_handle(handle1));
  hw::JointHandle handle2{JOINT2_NAME, FOO_INTERFACE};
  EXPECT_EQ(hw::return_type::ERROR, robot_hw_.get_joint_handle(handle2));
}

TEST_F(TestJoints, can_get_registered_joints)
{
  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  EXPECT_THAT(robot_hw_.get_registered_joint_names(), ElementsAre(JOINT_NAME));
  const auto registered_jointss = robot_hw_.get_registered_joints();
  EXPECT_THAT(registered_jointss, SizeIs(1));
  const auto & joints_handle = registered_jointss[0];
  EXPECT_EQ(joints_handle.get_name(), JOINT_NAME);
  EXPECT_EQ(joints_handle.get_interface_name(), FOO_INTERFACE);

  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));
  EXPECT_THAT(robot_hw_.get_registered_joint_names(), ElementsAre(JOINT_NAME));

  EXPECT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT2_NAME, FOO_INTERFACE));
  EXPECT_THAT(
    robot_hw_.get_registered_joint_names(),
    UnorderedElementsAre(JOINT_NAME, JOINT2_NAME));
  EXPECT_THAT(
    robot_hw_.get_registered_joint_interface_names(JOINT_NAME),
    UnorderedElementsAre(FOO_INTERFACE, BAR_INTERFACE));
  EXPECT_THAT(
    robot_hw_.get_registered_joint_interface_names(JOINT2_NAME),
    UnorderedElementsAre(FOO_INTERFACE));

  std::vector<hw::JointHandle> handles =
  {{JOINT_NAME, FOO_INTERFACE}, {JOINT_NAME, BAR_INTERFACE},
    {JOINT2_NAME, FOO_INTERFACE}};

  for (auto & handle : handles) {
    EXPECT_EQ(hw::return_type::OK, robot_hw_.get_joint_handle(handle));
  }
}

TEST_F(TestJoints, set_get_works_on_registered_jointss)
{
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT2_NAME, FOO_INTERFACE));
  auto joints_handles = robot_hw_.get_registered_joints();

  for (auto & handle : joints_handles) {
    const auto new_value = handle.get_value() + 1.337;
    EXPECT_NO_THROW(handle.set_value(new_value));
    EXPECT_DOUBLE_EQ(handle.get_value(), new_value);
  }

  std::vector<hw::JointHandle> handles =
  {{JOINT_NAME, FOO_INTERFACE}, {JOINT_NAME, BAR_INTERFACE},
    {JOINT2_NAME, FOO_INTERFACE}};

  for (auto & handle : handles) {
    EXPECT_ANY_THROW(handle.get_value());
    EXPECT_ANY_THROW(handle.set_value(0.0));

    EXPECT_EQ(hw::return_type::OK, robot_hw_.get_joint_handle(handle));
    const auto new_value = handle.get_value() + 1.337;
    EXPECT_NO_THROW(handle.set_value(new_value));
    EXPECT_DOUBLE_EQ(handle.get_value(), new_value);
  }
}

TEST_F(TestJoints, can_get_registered_joints_of_interface)
{
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, FOO_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT_NAME, BAR_INTERFACE));
  ASSERT_EQ(hw::return_type::OK, robot_hw_.register_joint(JOINT2_NAME, FOO_INTERFACE));

  std::vector<hw::JointHandle> handles1;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_joint_handles(handles1, FOO_INTERFACE));
  ASSERT_EQ(handles1.size(), 2ul);
  for (const auto & handle : handles1) {
    ASSERT_EQ(handle.get_interface_name(), FOO_INTERFACE);
  }

  std::vector<hw::JointHandle> handles2;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_joint_handles(handles2, BAR_INTERFACE));
  ASSERT_EQ(handles2.size(), 1ul);
  for (const auto & handle : handles2) {
    ASSERT_EQ(handle.get_interface_name(), BAR_INTERFACE);
  }

  std::vector<hw::JointHandle> handles3;
  ASSERT_EQ(hw::return_type::OK, robot_hw_.get_joint_handles(handles3, "NoInterface"));
  ASSERT_TRUE(handles3.empty());
}
