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

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/simple_transmission.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using transmission_interface::ActuatorHandle;
using transmission_interface::Exception;
using transmission_interface::JointHandle;
using transmission_interface::SimpleTransmission;

using testing::DoubleNear;

// Floating-point value comparison threshold
const double EPS = 1e-5;

TEST(PreconditionsTest, ExceptionThrownWithInvalidParameters)
{
  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(SimpleTransmission(0.0), Exception);
  EXPECT_THROW(SimpleTransmission(0.0, 1.0), Exception);
  EXPECT_THROW(SimpleTransmission(0.0, -1.0), Exception);
}

TEST(PreconditionsTest, NoExceptionsWithValidParameters)
{
  EXPECT_NO_THROW(SimpleTransmission(1.0));
  EXPECT_NO_THROW(SimpleTransmission(1.0, 1.0));
  EXPECT_NO_THROW(SimpleTransmission(-1.0, 1.0));
  EXPECT_NO_THROW(SimpleTransmission(1.0, -1.0));
  EXPECT_NO_THROW(SimpleTransmission(-1.0, -1.0));
}

TEST(PreconditionsTest, AccessorValidation)
{
  SimpleTransmission trans(2.0, -1.0);

  EXPECT_EQ(1u, trans.num_actuators());
  EXPECT_EQ(1u, trans.num_joints());
  EXPECT_THAT(2.0, DoubleNear(trans.get_actuator_reduction(), EPS));
  EXPECT_THAT(-1.0, DoubleNear(trans.get_joint_offset(), EPS));
}

TEST(PreconditionsTest, ConfigureFailsWithInvalidHandles)
{
  SimpleTransmission trans(2.0, -1.0);

  hardware_interface::InterfaceInfo interface_info;
  interface_info.name = HW_IF_POSITION;
  std::shared_ptr<ActuatorHandle> actuator_handle = std::make_shared<ActuatorHandle>(
    hardware_interface::InterfaceDescription("act1", interface_info));
  std::shared_ptr<ActuatorHandle> actuator2_handle = std::make_shared<ActuatorHandle>(
    hardware_interface::InterfaceDescription("act2", interface_info));
  std::shared_ptr<JointHandle> joint_handle = std::make_shared<JointHandle>(
    hardware_interface::InterfaceDescription("joint1", interface_info));
  std::shared_ptr<JointHandle> joint2_handle = std::make_shared<JointHandle>(
    hardware_interface::InterfaceDescription("joint2", interface_info));

  EXPECT_THROW(trans.configure({}, {}), transmission_interface::Exception);
  EXPECT_THROW(trans.configure({joint_handle}, {}), transmission_interface::Exception);
  EXPECT_THROW(trans.configure({}, {actuator_handle}), transmission_interface::Exception);

  EXPECT_THROW(
    trans.configure({joint_handle}, {actuator_handle, actuator2_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({joint_handle, joint2_handle}, {actuator_handle}),
    transmission_interface::Exception);

  auto invalid_actuator_handle = nullptr;
  auto invalid_joint_handle = nullptr;
  EXPECT_THROW(
    trans.configure({invalid_joint_handle}, {invalid_actuator_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({}, {actuator_handle, invalid_actuator_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(trans.configure({}, {actuator_handle}), transmission_interface::Exception);
  EXPECT_THROW(trans.configure({joint_handle}, {}), transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({invalid_joint_handle}, {actuator_handle}), transmission_interface::Exception);
}

/// Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public ::testing::TestWithParam<SimpleTransmission>
{
protected:
  /**
   * \param[in] trans Transmission instance.
   * \param[in] interface_name The joint/actuator interface used
   * \param[in] ref_val Reference value that will be transformed with the respective forward
   * and inverse transmission transformations.
   */
  void testIdentityMap(
    SimpleTransmission & trans, const std::string & interface_name, const double ref_val)
  {
    // Effort interface
    {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = interface_name;
      auto actuator_handle = std::make_shared<ActuatorHandle>(
        hardware_interface::InterfaceDescription("act1", interface_info));
      auto joint_handle = std::make_shared<JointHandle>(
        hardware_interface::InterfaceDescription("joint1", interface_info));
      const std::vector<ActuatorHandle::SharedPtr> actuator_handles({actuator_handle});
      const std::vector<JointHandle::SharedPtr> joint_handles({joint_handle});
      trans.configure(joint_handles, actuator_handles);

      std::ignore = actuator_handle->set_value(ref_val);

      trans.actuator_to_joint();
      trans.joint_to_actuator();
      EXPECT_THAT(ref_val, DoubleNear(actuator_handle->get_optional().value(), EPS));
    }
  }
};

TEST_P(BlackBoxTest, IdentityMap)
{
  // Transmission instance
  SimpleTransmission trans = GetParam();

  // Test transmission for positive, zero, and negative inputs
  testIdentityMap(trans, HW_IF_POSITION, 1.0);
  testIdentityMap(trans, HW_IF_POSITION, 0.0);
  testIdentityMap(trans, HW_IF_POSITION, -1.0);

  testIdentityMap(trans, HW_IF_VELOCITY, 1.0);
  testIdentityMap(trans, HW_IF_VELOCITY, 0.0);
  testIdentityMap(trans, HW_IF_VELOCITY, -1.0);

  testIdentityMap(trans, HW_IF_EFFORT, 1.0);
  testIdentityMap(trans, HW_IF_EFFORT, 0.0);
  testIdentityMap(trans, HW_IF_EFFORT, -1.0);
}

INSTANTIATE_TEST_SUITE_P(
  IdentityMap, BlackBoxTest,
  ::testing::Values(
    SimpleTransmission(10.0), SimpleTransmission(-10.0), SimpleTransmission(10.0, 1.0),
    SimpleTransmission(10.0, -1.0), SimpleTransmission(-10.0, 1.0),
    SimpleTransmission(-10.0, -1.0)));

class WhiteBoxTest : public ::testing::Test
{
};

TEST_F(WhiteBoxTest, MoveJoint)
{
  // NOTE: We only test the actuator->joint map
  // as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  SimpleTransmission trans(10.0, 1.0);

  // Effort interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_EFFORT;
    auto actuator_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto joint_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    const std::vector<ActuatorHandle::SharedPtr> actuator_handles({actuator_handle});
    const std::vector<JointHandle::SharedPtr> joint_handles({joint_handle});

    std::ignore = actuator_handle->set_value(1.0);
    trans.configure(joint_handles, actuator_handles);

    trans.actuator_to_joint();
    EXPECT_THAT(10.0, DoubleNear(joint_handle->get_optional().value(), EPS));
  }

  // Velocity interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_VELOCITY;
    auto actuator_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto joint_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    const std::vector<ActuatorHandle::SharedPtr> actuator_handles({actuator_handle});
    const std::vector<JointHandle::SharedPtr> joint_handles({joint_handle});

    std::ignore = actuator_handle->set_value(1.0);
    trans.configure(joint_handles, actuator_handles);

    trans.actuator_to_joint();
    EXPECT_THAT(0.1, DoubleNear(joint_handle->get_optional().value(), EPS));
  }

  // Position interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto actuator_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto joint_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    const std::vector<ActuatorHandle::SharedPtr> actuator_handles({actuator_handle});
    const std::vector<JointHandle::SharedPtr> joint_handles({joint_handle});

    std::ignore = actuator_handle->set_value(1.0);
    trans.configure(joint_handles, actuator_handles);

    trans.actuator_to_joint();
    EXPECT_THAT(1.1, DoubleNear(joint_handle->get_optional().value(), EPS));
  }

  // Mismatched interface is ignored
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto actuator_handle_pos = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto joint_handle_pos = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    interface_info.name = HW_IF_VELOCITY;
    auto actuator_handle_vel = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto joint_handle_vel = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));

    double unique_value = 13.37;
    std::ignore = joint_handle_vel->set_value(unique_value);
    std::ignore = actuator_handle_vel->set_value(unique_value);

    trans.configure({joint_handle_pos, joint_handle_vel}, {actuator_handle_pos});
    trans.actuator_to_joint();
    EXPECT_THAT(unique_value, DoubleNear(joint_handle_vel->get_optional().value(), EPS));
    EXPECT_THAT(unique_value, DoubleNear(actuator_handle_vel->get_optional().value(), EPS));

    trans.configure({joint_handle_pos}, {actuator_handle_pos, actuator_handle_vel});
    trans.actuator_to_joint();
    EXPECT_THAT(unique_value, DoubleNear(joint_handle_vel->get_optional().value(), EPS));
    EXPECT_THAT(unique_value, DoubleNear(actuator_handle_vel->get_optional().value(), EPS));
  }
}
