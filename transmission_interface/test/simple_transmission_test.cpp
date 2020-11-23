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

#include <transmission_interface/simple_transmission.h>

#include <gmock/gmock.h>
#include <vector>
#include <string>

using std::vector;
using transmission_interface::SimpleTransmission;
using transmission_interface::Exception;
using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(SimpleTransmission(0.0), Exception);
  EXPECT_THROW(SimpleTransmission(0.0, 1.0), Exception);
  EXPECT_THROW(SimpleTransmission(0.0, -1.0), Exception);

  // Valid instance creation
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
  EXPECT_EQ(2.0, trans.get_actuator_reduction());
  EXPECT_EQ(-1.0, trans.get_joint_offset());
}

// #ifndef NDEBUG
// NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
// TEST(PreconditionsTest, AssertionTriggering)
// {
//   // Create input/output transmission data
//   double a_val = 0.0;
//   double j_val = 0.0;

//   ActuatorData a_good_data;
//   a_good_data.position = vector<double*>(1, &a_val);
//   a_good_data.velocity = vector<double*>(1, &a_val);
//   a_good_data.effort   = vector<double*>(1, &a_val);

//   JointData j_good_data;
//   j_good_data.position = vector<double*>(1, &j_val);
//   j_good_data.velocity = vector<double*>(1, &j_val);
//   j_good_data.effort   = vector<double*>(1, &j_val);

//   ActuatorData a_bad_data;
//   a_bad_data.position = vector<double*>(1);
//   a_bad_data.velocity = vector<double*>(1);
//   a_bad_data.effort   = vector<double*>(1);

//   JointData j_bad_data;
//   j_bad_data.position = vector<double*>(1);
//   j_bad_data.velocity = vector<double*>(1);
//   j_bad_data.effort   = vector<double*>(1);

//   ActuatorData a_bad_size;
//   JointData    j_bad_size;

//   // Transmission instance
//   SimpleTransmission trans = SimpleTransmission(1.0);

//   // Data with invalid pointers should trigger an assertion
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_bad_data,  j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_good_data, j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_bad_data,  j_good_data), ".*");

//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_bad_data,  j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_good_data, j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_bad_data,  j_good_data), ".*");

//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_bad_data,  j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_good_data, j_bad_data),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_bad_data,  j_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_bad_data,  a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_good_data, a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_bad_data,  a_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_bad_data,  a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_good_data, a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_bad_data,  a_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_bad_data,  a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_good_data, a_bad_data),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_bad_data,  a_good_data), ".*");

//   // Wrong parameter sizes should trigger an assertion
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_bad_size,  j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_good_data, j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointEffort(a_bad_size,  j_good_data), ".*");

//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_bad_size,  j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_good_data, j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointVelocity(a_bad_size,  j_good_data), ".*");

//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_bad_size,  j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_good_data, j_bad_size),  ".*");
//   EXPECT_DEATH(trans.actuator_to_jointPosition(a_bad_size,  j_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_bad_size,  a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_good_data, a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorEffort(j_bad_size,  a_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_bad_size,  a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_good_data, a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorVelocity(j_bad_size,  a_good_data), ".*");

//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_bad_size,  a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_good_data, a_bad_size),  ".*");
//   EXPECT_DEATH(trans.joint_to_actuatorPosition(j_bad_size,  a_good_data), ".*");
// }
// #endif // NDEBUG


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

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup,
  public ::testing::TestWithParam<SimpleTransmission>
{
protected:
  /// \param trans Transmission instance.
  /// \param interface_name The joint/actuator interface used
  /// \param ref_val Reference value that will be transformed with the respective forward
  /// and inverse transmission transformations.
  void testIdentityMap(
    SimpleTransmission & trans, const std::string & interface_name,
    const double ref_val)
  {
    // Effort interface
    {
      auto actuator_handle = ActuatorHandle("joint1", interface_name, &a_val);
      auto joint_handle = JointHandle("joint1", interface_name, &j_val);
      trans.configure({joint_handle}, {actuator_handle});

      a_val = ref_val;

      trans.actuator_to_joint();
      trans.joint_to_actuator();
      EXPECT_NEAR(ref_val, a_val, EPS);
    }
  }
};

TEST_P(BlackBoxTest, IdentityMap)
{
  // Transmission instance
  SimpleTransmission trans = GetParam();

  // Test transmission for positive, zero, and negative inputs
  testIdentityMap(trans, "position", 1.0);
  reset_values();
  testIdentityMap(trans, "position", 0.0);
  reset_values();
  testIdentityMap(trans, "position", -1.0);
  reset_values();

  testIdentityMap(trans, "velocity", 1.0);
  reset_values();
  testIdentityMap(trans, "velocity", 0.0);
  reset_values();
  testIdentityMap(trans, "velocity", -1.0);
  reset_values();

  testIdentityMap(trans, "effort", 1.0);
  reset_values();
  testIdentityMap(trans, "effort", 0.0);
  reset_values();
  testIdentityMap(trans, "effort", -1.0);
}

INSTANTIATE_TEST_CASE_P(
  IdentityMap,
  BlackBoxTest,
  ::testing::Values(
    SimpleTransmission(10.0),
    SimpleTransmission(-10.0),
    SimpleTransmission(10.0, 1.0),
    SimpleTransmission(10.0, -1.0),
    SimpleTransmission(-10.0, 1.0),
    SimpleTransmission(-10.0, -1.0)));

class WhiteBoxTest : public TransmissionSetup,
  public ::testing::Test {};

TEST_F(WhiteBoxTest, MoveJoint)
{
  // NOTE: We only test the actuator->joint map
  // as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  SimpleTransmission trans(10.0, 1.0);

  a_val = 1.0;

  // Effort interface
  {
    auto actuator_handle = ActuatorHandle("joint1", "effort", &a_val);
    auto joint_handle = JointHandle("joint1", "effort", &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_NEAR(10.0, j_val, EPS);
  }

  // Velocity interface
  {
    auto actuator_handle = ActuatorHandle("joint1", "velocity", &a_val);
    auto joint_handle = JointHandle("joint1", "velocity", &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_NEAR(0.1, j_val, EPS);
  }

  // Position interface
  {
    auto actuator_handle = ActuatorHandle("joint1", "position", &a_val);
    auto joint_handle = JointHandle("joint1", "position", &j_val);
    trans.configure({joint_handle}, {actuator_handle});

    trans.actuator_to_joint();
    EXPECT_NEAR(1.1, j_val, EPS);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
