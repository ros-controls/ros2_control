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
#include "random_generator_utils.hpp"
#include "transmission_interface/differential_transmission.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using testing::DoubleNear;
using transmission_interface::ActuatorHandle;
using transmission_interface::DifferentialTransmission;
using transmission_interface::Exception;
using transmission_interface::JointHandle;
// Floating-point value comparison threshold
const double EPS = 1e-5;

TEST(PreconditionsTest, ExceptionThrowing)
{
  const std::vector<double> reduction_good = {1.0, 1.0};
  const std::vector<double> reduction_bad1 = {0.0, 0.0};
  const std::vector<double> reduction_bad2 = {1.0, 0.0};
  const std::vector<double> reduction_bad3 = {0.0, 1.0};
  const std::vector<double> offset_good = {1.0, 1.0};

  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(DifferentialTransmission(reduction_bad1, reduction_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_bad2, reduction_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_bad3, reduction_good), Exception);

  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad1), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad2), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad3), Exception);

  EXPECT_THROW(DifferentialTransmission(reduction_bad1, reduction_good, offset_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_bad2, reduction_good, offset_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_bad3, reduction_good, offset_good), Exception);

  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad1, offset_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad2, offset_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad3, offset_good), Exception);

  // Invalid instance creation: Wrong parameter sizes
  const std::vector<double> reduction_bad_size = {1.0};
  const std::vector<double> & offset_bad_size = reduction_bad_size;
  EXPECT_THROW(DifferentialTransmission(reduction_bad_size, reduction_good), Exception);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad_size), Exception);
  EXPECT_THROW(
    DifferentialTransmission(reduction_good, reduction_good, offset_bad_size), Exception);

  // Valid instance creation
  EXPECT_NO_THROW(DifferentialTransmission(reduction_good, reduction_good));
  EXPECT_NO_THROW(DifferentialTransmission(reduction_good, reduction_good, offset_good));
}

TEST(PreconditionsTest, AccessorValidation)
{
  std::vector<double> act_reduction = {2.0, -2.0};
  std::vector<double> jnt_reduction = {4.0, -4.0};
  std::vector<double> jnt_offset = {1.0, -1.0};

  DifferentialTransmission trans(act_reduction, jnt_reduction, jnt_offset);

  EXPECT_EQ(2u, trans.num_actuators());
  EXPECT_EQ(2u, trans.num_joints());
  EXPECT_THAT(2.0, DoubleNear(trans.get_actuator_reduction()[0], EPS));
  EXPECT_THAT(-2.0, DoubleNear(trans.get_actuator_reduction()[1], EPS));
  EXPECT_THAT(4.0, DoubleNear(trans.get_joint_reduction()[0], EPS));
  EXPECT_THAT(-4.0, DoubleNear(trans.get_joint_reduction()[1], EPS));
  EXPECT_THAT(1.0, DoubleNear(trans.get_joint_offset()[0], EPS));
  EXPECT_THAT(-1.0, DoubleNear(trans.get_joint_offset()[1], EPS));
}

void testConfigureWithBadHandles(std::string interface_name)
{
  DifferentialTransmission trans({1.0, 1.0}, {1.0, 1.0});

  hardware_interface::InterfaceInfo interface_info;
  interface_info.name = interface_name;

  auto a1_handle = std::make_shared<ActuatorHandle>(
    hardware_interface::InterfaceDescription("act1", interface_info));
  auto a2_handle = std::make_shared<ActuatorHandle>(
    hardware_interface::InterfaceDescription("act2", interface_info));
  auto a3_handle = std::make_shared<ActuatorHandle>(
    hardware_interface::InterfaceDescription("act3", interface_info));
  auto j1_handle = std::make_shared<JointHandle>(
    hardware_interface::InterfaceDescription("joint1", interface_info));
  auto j2_handle = std::make_shared<JointHandle>(
    hardware_interface::InterfaceDescription("joint2", interface_info));
  auto j3_handle = std::make_shared<JointHandle>(
    hardware_interface::InterfaceDescription("joint3", interface_info));
  auto invalid_a1_handle = nullptr;
  auto invalid_j1_handle = nullptr;

  EXPECT_THROW(trans.configure({}, {}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({j1_handle, j2_handle}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle, j3_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle, a3_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle, j3_handle}, {a1_handle, a2_handle, a3_handle}),
    Exception);
  EXPECT_THROW(trans.configure({j1_handle, j2_handle}, {invalid_a1_handle, a2_handle}), Exception);
  EXPECT_THROW(trans.configure({invalid_j1_handle, j2_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({invalid_j1_handle, j2_handle}, {invalid_a1_handle, a2_handle}), Exception);
}

TEST(ConfigureTest, FailsWithBadHandles)
{
  testConfigureWithBadHandles(HW_IF_POSITION);
  testConfigureWithBadHandles(HW_IF_VELOCITY);
  testConfigureWithBadHandles(HW_IF_EFFORT);
}

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public ::testing::Test
{
protected:
  const double EXTREMAL_VALUE = 1337.1337;
  /// \param trans Transmission instance.
  /// \param ref_val Reference value that will be transformed with the respective forward
  /// and inverse transmission transformations.
  /// \param interface_name The name of the interface to test, position, velocity, etc.
  void testIdentityMap(
    DifferentialTransmission & trans, const std::vector<double> & ref_val,
    const std::string & interface_name)
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;

    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));

    // set actuator values to reference
    std::ignore = a1_handle->set_value(ref_val[0]);
    std::ignore = a2_handle->set_value(ref_val[1]);
    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});

    // actuator->joint->actuator roundtrip
    // but we also set actuator values to an extremal value
    // to ensure joint_to_actuator is not a no-op
    trans.actuator_to_joint();
    std::ignore = a1_handle->set_value(EXTREMAL_VALUE);
    std::ignore = a2_handle->set_value(EXTREMAL_VALUE);
    trans.joint_to_actuator();
    EXPECT_THAT(ref_val[0], DoubleNear(a1_handle->get_optional().value(), EPS));
    EXPECT_THAT(ref_val[1], DoubleNear(a2_handle->get_optional().value(), EPS));
  }

  // Generate a set of transmission instances
  // with random combinations of actuator/joint reduction and joint offset.
  static std::vector<DifferentialTransmission> createTestInstances(
    const vector<DifferentialTransmission>::size_type size)
  {
    std::vector<DifferentialTransmission> out;
    out.reserve(size);
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);

    while (out.size() < size)
    {
      try
      {
        DifferentialTransmission trans(
          randomVector(2, rand_gen), randomVector(2, rand_gen), randomVector(2, rand_gen));
        out.push_back(trans);
      }
      catch (const Exception &)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator,
        // construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};

TEST_F(BlackBoxTest, IdentityMap)
{
  // Transmission instances
  // NOTE: Magic value
  auto transmission_test_instances = createTestInstances(100);

  // Test different transmission configurations...
  for (auto && transmission : transmission_test_instances)
  {
    // ...and for each transmission, different input values
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);
    // NOTE: Magic value
    const unsigned int input_value_trials = 100;
    for (unsigned int i = 0; i < input_value_trials; ++i)
    {
      vector<double> input_value = randomVector(2, rand_gen);
      // Test each interface type separately
      testIdentityMap(transmission, input_value, HW_IF_POSITION);
      testIdentityMap(transmission, input_value, HW_IF_VELOCITY);
      testIdentityMap(transmission, input_value, HW_IF_EFFORT);
    }
  }
}

class WhiteBoxTest : public ::testing::Test
{
};

TEST_F(WhiteBoxTest, DontMoveJoints)
{
  std::vector<double> actuator_reduction = {10.0, 10.0};
  std::vector<double> joint_reduction = {2.0, 2.0};
  std::vector<double> joint_offset = {1.0, 1.0};

  DifferentialTransmission trans(actuator_reduction, joint_reduction, joint_offset);
  const double a1_value = 0.0;
  const double a2_value = 0.0;

  // Effort interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_EFFORT;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Velocity interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_VELOCITY;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Position interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(joint_offset[0], DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(joint_offset[1], DoubleNear(j2_handle->get_optional().value(), EPS));
  }
}

TEST_F(WhiteBoxTest, MoveFirstJointOnly)
{
  std::vector<double> actuator_reduction = {10.0, 10.0};
  std::vector<double> joint_reduction = {2.0, 2.0};

  DifferentialTransmission trans(actuator_reduction, joint_reduction);

  // Actuator input (used for effort, velocity and position)
  const double a1_value = 10.0;
  const double a2_value = 10.0;

  // Effort interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_EFFORT;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(400.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Velocity interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_VELOCITY;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.5, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Position interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.5, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }
}

TEST_F(WhiteBoxTest, MoveSecondJointOnly)
{
  std::vector<double> actuator_reduction = {10.0, 10.0};
  std::vector<double> joint_reduction = {2.0, 2.0};

  DifferentialTransmission trans(actuator_reduction, joint_reduction);

  // Actuator input (used for effort, velocity and position)
  const double a1_value = 10.0;
  const double a2_value = -10.0;

  // Effort interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_EFFORT;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(400.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Velocity interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_VELOCITY;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.5, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Position interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(0.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.5, DoubleNear(j2_handle->get_optional().value(), EPS));
  }
}

TEST_F(WhiteBoxTest, MoveBothJoints)
{
  // NOTE: We only test the actuator->joint map,
  // as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  std::vector<double> actuator_reduction = {10.0, -20.0};
  std::vector<double> joint_reduction = {-2.0, 4.0};
  std::vector<double> joint_offset = {-2.0, 4.0};

  DifferentialTransmission trans(actuator_reduction, joint_reduction, joint_offset);

  // Actuator input (used for effort, velocity and position)
  const double a1_value = 3.0;
  const double a2_value = 5.0;

  // Effort interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_EFFORT;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(140.0, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(520.0, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Velocity interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_VELOCITY;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(-0.01250, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(0.06875, DoubleNear(j2_handle->get_optional().value(), EPS));
  }

  // Position interface
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = HW_IF_POSITION;
    auto a1_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act1", interface_info));
    auto a2_handle = std::make_shared<ActuatorHandle>(
      hardware_interface::InterfaceDescription("act2", interface_info));
    auto j1_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint1", interface_info));
    auto j2_handle = std::make_shared<JointHandle>(
      hardware_interface::InterfaceDescription("joint2", interface_info));
    std::ignore = a1_handle->set_value(a1_value);
    std::ignore = a2_handle->set_value(a2_value);

    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle});
    trans.actuator_to_joint();
    EXPECT_THAT(-2.01250, DoubleNear(j1_handle->get_optional().value(), EPS));
    EXPECT_THAT(4.06875, DoubleNear(j2_handle->get_optional().value(), EPS));
  }
}
