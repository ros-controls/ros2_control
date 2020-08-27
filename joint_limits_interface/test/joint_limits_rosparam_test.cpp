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
#include <joint_limits_interface/joint_limits_rosparam.h>

using std::string;
using namespace joint_limits_interface;


TEST(JointLimitsRosParamTest, GetJointLimits)
{
  using namespace joint_limits_interface;

  ros::NodeHandle nh("test");

  // Invalid specification
  {
    JointLimits limits;
    EXPECT_FALSE(getJointLimits("~bad_joint", nh, limits));
    EXPECT_FALSE(getJointLimits("unknown_joint", nh, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Get full specification from parameter server
  {
    JointLimits limits;
    EXPECT_TRUE(getJointLimits("foo_joint", nh, limits));

    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_EQ(0.0, limits.min_position);
    EXPECT_EQ(1.0, limits.max_position);

    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_EQ(2.0, limits.max_velocity);

    EXPECT_TRUE(limits.has_acceleration_limits);
    EXPECT_EQ(5.0, limits.max_acceleration);

    EXPECT_TRUE(limits.has_jerk_limits);
    EXPECT_EQ(100.0, limits.max_jerk);

    EXPECT_TRUE(limits.has_effort_limits);
    EXPECT_EQ(20.0, limits.max_effort);

    EXPECT_FALSE(limits.angle_wraparound);
  }

  // Specifying flags but not values should set nothing
  {
    JointLimits limits;
    EXPECT_TRUE(getJointLimits("yinfoo_joint", nh, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Specifying values but not flags should set nothing
  {
    JointLimits limits;
    EXPECT_TRUE(getJointLimits("yangfoo_joint", nh, limits));

    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
  }

  // Disable already set values
  {
    JointLimits limits;
    EXPECT_TRUE(getJointLimits("foo_joint", nh, limits));
    EXPECT_TRUE(limits.has_position_limits);
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_TRUE(limits.has_acceleration_limits);
    EXPECT_TRUE(limits.has_jerk_limits);
    EXPECT_TRUE(limits.has_effort_limits);

    EXPECT_TRUE(getJointLimits("antifoo_joint", nh, limits));
    EXPECT_FALSE(limits.has_position_limits);
    EXPECT_FALSE(limits.has_velocity_limits);
    EXPECT_FALSE(limits.has_acceleration_limits);
    EXPECT_FALSE(limits.has_jerk_limits);
    EXPECT_FALSE(limits.has_effort_limits);
    EXPECT_TRUE(limits.angle_wraparound);
  }

  // Incomplete position limits specification does not get loaded
  {
    JointLimits limits;
    EXPECT_TRUE(getJointLimits("baz_joint", nh, limits));

    EXPECT_FALSE(limits.has_position_limits);
  }

  // Override only one field, leave all others unchanged
  {
    JointLimits limits, limits_ref;
    EXPECT_TRUE(getJointLimits("bar_joint", nh, limits));

    EXPECT_EQ(limits_ref.has_position_limits, limits.has_position_limits);
    EXPECT_EQ(limits_ref.min_position, limits.min_position);
    EXPECT_EQ(limits_ref.max_position, limits.max_position);

    // Max velocity is overridden
    EXPECT_NE(limits_ref.has_velocity_limits, limits.has_velocity_limits);
    EXPECT_NE(limits_ref.max_velocity, limits.max_velocity);
    EXPECT_TRUE(limits.has_velocity_limits);
    EXPECT_EQ(2.0, limits.max_velocity);

    EXPECT_EQ(limits_ref.has_acceleration_limits, limits.has_acceleration_limits);
    EXPECT_EQ(limits_ref.max_acceleration, limits.max_acceleration);

    EXPECT_EQ(limits_ref.has_jerk_limits, limits.has_jerk_limits);
    EXPECT_EQ(limits_ref.has_jerk_limits, limits.max_jerk);

    EXPECT_EQ(limits_ref.has_effort_limits, limits.has_effort_limits);
    EXPECT_EQ(limits_ref.max_effort, limits.max_effort);
  }
}

TEST(JointLimitsRosParamTest, GetSoftJointLimits)
{
  using namespace joint_limits_interface;

  ros::NodeHandle nh("test");

  // Invalid specification
  {
    SoftJointLimits soft_limits;
    EXPECT_FALSE(getSoftJointLimits("~bad_joint", nh, soft_limits));
    EXPECT_FALSE(getSoftJointLimits("unknown_joint", nh, soft_limits));
  }

  // Get full specification from parameter server
  {
    SoftJointLimits soft_limits;
    EXPECT_TRUE(getSoftJointLimits("foo_joint", nh, soft_limits));

    EXPECT_EQ(10.0, soft_limits.k_position);
    EXPECT_EQ(20.0, soft_limits.k_velocity);
    EXPECT_EQ(0.1, soft_limits.min_position);
    EXPECT_EQ(0.9, soft_limits.max_position);
  }

  // Skip parsing soft limits if has_soft_limits is false
  {
    SoftJointLimits soft_limits, soft_limits_ref;
    EXPECT_FALSE(getSoftJointLimits("foobar_joint", nh, soft_limits));
    EXPECT_EQ(soft_limits.k_position, soft_limits_ref.k_position);
    EXPECT_EQ(soft_limits.k_velocity, soft_limits_ref.k_velocity);
    EXPECT_EQ(soft_limits.min_position, soft_limits_ref.min_position);
    EXPECT_EQ(soft_limits.max_position, soft_limits_ref.max_position);
  }

  // Incomplete soft limits specification does not get loaded
  {
    SoftJointLimits soft_limits, soft_limits_ref;
    EXPECT_FALSE(getSoftJointLimits("barbaz_joint", nh, soft_limits));
    EXPECT_EQ(soft_limits.k_position, soft_limits_ref.k_position);
    EXPECT_EQ(soft_limits.k_velocity, soft_limits_ref.k_velocity);
    EXPECT_EQ(soft_limits.min_position, soft_limits_ref.min_position);
    EXPECT_EQ(soft_limits.max_position, soft_limits_ref.max_position);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "JointLimitsRosparamTestNode");
  return RUN_ALL_TESTS();
}
