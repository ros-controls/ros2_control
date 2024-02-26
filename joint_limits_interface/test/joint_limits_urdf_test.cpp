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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>
#include <string>

#include <joint_limits_interface/joint_limits_urdf.hpp>

class JointLimitsUrdfTest : public ::testing::Test
{
public:
  JointLimitsUrdfTest()
  {
    urdf_limits.reset(new urdf::JointLimits);
    urdf_limits->effort = 8.0;
    urdf_limits->velocity = 2.0;
    urdf_limits->lower = -1.0;
    urdf_limits->upper = 1.0;

    urdf_safety.reset(new urdf::JointSafety);
    urdf_safety->k_position = 20.0;
    urdf_safety->k_velocity = 40.0;
    urdf_safety->soft_lower_limit = -0.8;
    urdf_safety->soft_upper_limit = 0.8;

    urdf_joint.reset(new urdf::Joint);
    urdf_joint->limits = urdf_limits;
    urdf_joint->safety = urdf_safety;

    urdf_joint->type = urdf::Joint::UNKNOWN;
  }

protected:
  urdf::JointLimitsSharedPtr urdf_limits;
  urdf::JointSafetySharedPtr urdf_safety;
  urdf::JointSharedPtr urdf_joint;
};

TEST_F(JointLimitsUrdfTest, GetJointLimits)
{
  // Unset URDF joint
  {
    joint_limits_interface::JointLimits limits;
    urdf::JointSharedPtr urdf_joint_bad;
    EXPECT_FALSE(getJointLimits(urdf_joint_bad, limits));
  }

  // Unset URDF limits
  {
    joint_limits_interface::JointLimits limits;
    urdf::JointSharedPtr urdf_joint_bad(new urdf::Joint);
    EXPECT_FALSE(getJointLimits(urdf_joint_bad, limits));
  }

  // Valid URDF joint, CONTINUOUS type
  {
    urdf_joint->type = urdf::Joint::CONTINUOUS;

    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits(urdf_joint, limits));

    // Position
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_TRUE(limits.angle_wraparound);

    // Velocity
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->velocity, limits.max_velocity);

    // Acceleration
    EXPECT_FALSE(limits.has_acceleration_limits);

    // Effort
    EXPECT_TRUE(limits.has_effort_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->effort, limits.max_effort);
  }

  // Valid URDF joint, REVOLUTE type
  {
    urdf_joint->type = urdf::Joint::REVOLUTE;

    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits(urdf_joint, limits));

    // Position
    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->lower, limits.min_position);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->upper, limits.max_position);
    EXPECT_FALSE(limits.angle_wraparound);

    // Velocity
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->velocity, limits.max_velocity);

    // Acceleration
    EXPECT_FALSE(limits.has_acceleration_limits);

    // Effort
    EXPECT_TRUE(limits.has_effort_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->effort, limits.max_effort);
  }

  // Valid URDF joint, PRISMATIC type
  {
    urdf_joint->type = urdf::Joint::PRISMATIC;

    joint_limits_interface::JointLimits limits;
    EXPECT_TRUE(getJointLimits(urdf_joint, limits));

    // Position
    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->lower, limits.min_position);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->upper, limits.max_position);
    EXPECT_FALSE(limits.angle_wraparound);

    // Velocity
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->velocity, limits.max_velocity);

    // Acceleration
    EXPECT_FALSE(limits.has_acceleration_limits);

    // Effort
    EXPECT_TRUE(limits.has_effort_limits);
    EXPECT_DOUBLE_EQ(urdf_joint->limits->effort, limits.max_effort);
  }
}

TEST_F(JointLimitsUrdfTest, GetSoftJointLimits)
{
  // Unset URDF joint
  {
    joint_limits_interface::SoftJointLimits soft_limits;
    urdf::JointSharedPtr urdf_joint_bad;
    EXPECT_FALSE(getSoftJointLimits(urdf_joint_bad, soft_limits));
  }

  // Unset URDF limits
  {
    joint_limits_interface::SoftJointLimits soft_limits;
    urdf::JointSharedPtr urdf_joint_bad(new urdf::Joint);
    EXPECT_FALSE(getSoftJointLimits(urdf_joint_bad, soft_limits));
  }

  // Valid URDF joint
  {
    joint_limits_interface::SoftJointLimits soft_limits;
    EXPECT_TRUE(getSoftJointLimits(urdf_joint, soft_limits));

    // Soft limits
    EXPECT_DOUBLE_EQ(urdf_joint->safety->soft_lower_limit, soft_limits.min_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->soft_upper_limit, soft_limits.max_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->k_position, soft_limits.k_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->k_velocity, soft_limits.k_velocity);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
