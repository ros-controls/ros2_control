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
using std::vector;
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
  double dummy;

  auto actuator_handle = ActuatorHandle("act1", HW_IF_POSITION, &dummy);
  auto actuator2_handle = ActuatorHandle("act2", HW_IF_POSITION, &dummy);
  auto joint_handle = JointHandle("joint1", HW_IF_POSITION, &dummy);
  auto joint2_handle = JointHandle("joint2", HW_IF_POSITION, &dummy);

  EXPECT_THROW(trans.configure({}, {}), transmission_interface::Exception);
  EXPECT_THROW(trans.configure({joint_handle}, {}), transmission_interface::Exception);
  EXPECT_THROW(trans.configure({}, {actuator_handle}), transmission_interface::Exception);

  EXPECT_THROW(
    trans.configure({joint_handle}, {actuator_handle, actuator2_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({joint_handle, joint2_handle}, {actuator_handle}),
    transmission_interface::Exception);

  auto invalid_actuator_handle = ActuatorHandle("act1", HW_IF_VELOCITY, nullptr);
  auto invalid_joint_handle = JointHandle("joint1", HW_IF_VELOCITY, nullptr);
  EXPECT_THROW(
    trans.configure({invalid_joint_handle}, {invalid_actuator_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({}, {actuator_handle, invalid_actuator_handle}),
    transmission_interface::Exception);
  EXPECT_THROW(
    trans.configure({invalid_joint_handle}, {actuator_handle}), transmission_interface::Exception);
}

class TransmissionSetup
{
protected:
  // Input/output transmission data
  double a_val = 0.0;
  double j_val = 0.0;

  void reset_values()
  {
    a_val = 0.0;
    j_val = 0.0;
  }
};

/// Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup, public ::testing::TestWithParam<SimpleTransmission>
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
      auto actuator_handle = ActuatorHandle("act1", interface_name, &a_val);
      auto joint_handle = JointHandle("joint1", interface_name, &j_val);
      trans.configure({joint_handle}, {actuator_handle});

      a_val = ref_val;

      trans.actuator_to_joint();
      trans.joint_to_actuator();
      EXPECT_THAT(ref_val, DoubleNear(a_val, EPS));
    }
  }
};

TEST_P(BlackBoxTest, IdentityMap)
{
  // Transmission instance
  SimpleTransmission trans = GetParam();

  // Test transmission for positive, zero, and negative inputs
  testIdentityMap(trans, HW_IF_POSITION, 1.0);
  reset_values();
  testIdentityMap(trans, HW_IF_POSITION, 0.0);
  reset_values();
  testIdentityMap(trans, HW_IF_POSITION, -1.0);
  reset_values();

  testIdentityMap(trans, HW_IF_VELOCITY, 1.0);
  reset_values();
  testIdentityMap(trans, HW_IF_VELOCITY, 0.0);
  reset_values();
  testIdentityMap(trans, HW_IF_VELOCITY, -1.0);
  reset_values();

  testIdentityMap(trans, HW_IF_EFFORT, 1.0);
  reset_values();
  testIdentityMap(trans, HW_IF_EFFORT, 0.0);
  reset_values();
  testIdentityMap(trans, HW_IF_EFFORT, -1.0);
}

INSTANTIATE_TEST_SUITE_P(
  IdentityMap, BlackBoxTest,
  ::testing::Values(
    SimpleTransmission(10.0), SimpleTransmission(-10.0), SimpleTransmission(10.0, 1.0),
    SimpleTransmission(10.0, -1.0), SimpleTransmission(-10.0, 1.0),
    SimpleTransmission(-10.0, -1.0)));

class WhiteBoxTest : public TransmissionSetup, public ::testing::Test
{
};

TEST_F(WhiteBoxTest, MoveJoint)
{
  // NOTE: We only test the actuator->joint map
  // as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  SimpleTransmission trans(10.0, 1.0);

  a_val = 1.0;

  // Effort interface
  {
    auto actuator_handle = ActuatorHandle("act1", HW_IF_EFFORT, &a_val);
    auto joint_handle = JointHandle("joint1", HW_IF_EFFORT, &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_THAT(10.0, DoubleNear(j_val, EPS));
  }

  // Velocity interface
  {
    auto actuator_handle = ActuatorHandle("act1", HW_IF_VELOCITY, &a_val);
    auto joint_handle = JointHandle("joint1", HW_IF_VELOCITY, &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_THAT(0.1, DoubleNear(j_val, EPS));
  }

  // Position interface
  {
    auto actuator_handle = ActuatorHandle("act1", HW_IF_POSITION, &a_val);
    auto joint_handle = JointHandle("joint1", HW_IF_POSITION, &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_THAT(1.1, DoubleNear(j_val, EPS));
  }

  // Mismatched interface is ignored
  {
    double unique_value = 13.37;

    auto actuator_handle = ActuatorHandle("act1", HW_IF_POSITION, &a_val);
    auto actuator_handle2 = ActuatorHandle("act1", HW_IF_VELOCITY, &unique_value);
    auto joint_handle = JointHandle("joint1", HW_IF_POSITION, &j_val);
    auto joint_handle2 = JointHandle("joint1", HW_IF_VELOCITY, &unique_value);

    trans.configure({joint_handle, joint_handle2}, {actuator_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(unique_value, DoubleNear(13.37, EPS));

    trans.configure({joint_handle}, {actuator_handle, actuator_handle2});
    trans.actuator_to_joint();
    EXPECT_THAT(unique_value, DoubleNear(13.37, EPS));
  }
}
