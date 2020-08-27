///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>
#include <joint_limits_interface/joint_limits_urdf.hpp>

using std::string;
using namespace joint_limits_interface;

class JointLimitsUrdfTest : public ::testing::Test
{
public:
  JointLimitsUrdfTest()
  {
    urdf_limits.reset(new urdf::JointLimits);
    urdf_limits->effort   =  8.0;
    urdf_limits->velocity =  2.0;
    urdf_limits->lower    = -1.0;
    urdf_limits->upper    =  1.0;

    urdf_safety.reset(new urdf::JointSafety);
    urdf_safety->k_position       = 20.0;
    urdf_safety->k_velocity       = 40.0;
    urdf_safety->soft_lower_limit = -0.8;
    urdf_safety->soft_upper_limit =  0.8;

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
    JointLimits limits;
    urdf::JointSharedPtr urdf_joint_bad;
    EXPECT_FALSE(getJointLimits(urdf_joint_bad, limits));
  }

  // Unset URDF limits
  {
    JointLimits limits;
    urdf::JointSharedPtr urdf_joint_bad(new urdf::Joint);
    EXPECT_FALSE(getJointLimits(urdf_joint_bad, limits));
  }

  // Valid URDF joint, CONTINUOUS type
  {
    urdf_joint->type = urdf::Joint::CONTINUOUS;

    JointLimits limits;
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

    JointLimits limits;
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

    JointLimits limits;
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
  using namespace joint_limits_interface;

  // Unset URDF joint
  {
    SoftJointLimits soft_limits;
    urdf::JointSharedPtr urdf_joint_bad;
    EXPECT_FALSE(getSoftJointLimits(urdf_joint_bad, soft_limits));
  }

  // Unset URDF limits
  {
    SoftJointLimits soft_limits;
    urdf::JointSharedPtr urdf_joint_bad(new urdf::Joint);
    EXPECT_FALSE(getSoftJointLimits(urdf_joint_bad, soft_limits));
  }

  // Valid URDF joint
  {
    SoftJointLimits soft_limits;
    EXPECT_TRUE(getSoftJointLimits(urdf_joint, soft_limits));

    // Soft limits
    EXPECT_DOUBLE_EQ(urdf_joint->safety->soft_lower_limit, soft_limits.min_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->soft_upper_limit, soft_limits.max_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->k_position,       soft_limits.k_position);
    EXPECT_DOUBLE_EQ(urdf_joint->safety->k_velocity,       soft_limits.k_velocity);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
