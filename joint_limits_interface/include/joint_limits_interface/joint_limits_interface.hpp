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

#ifndef JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_HPP_
#define JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_HPP_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <hardware_interface/joint_handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcppmath/clamp.hpp>

#include "joint_limits_interface/joint_limits.hpp"
#include "joint_limits_interface/joint_limits_interface_exception.hpp"

namespace joint_limits_interface
{
/**
 * The base class of limit handles for enforcing position, velocity, and effort limits of
 * an effort-controlled joint.
 */
class JointLimitHandle
{
public:
  JointLimitHandle()
  : prev_pos_(std::numeric_limits<double>::quiet_NaN()),
    prev_vel_(0.0),
    jposh_(hardware_interface::HW_IF_POSITION),
    jvelh_(hardware_interface::HW_IF_VELOCITY),
    jcmdh_("position_command")
  {
  }

  JointLimitHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const JointLimits & limits)
  : jposh_(jposh),
    jvelh_(hardware_interface::HW_IF_VELOCITY),
    jcmdh_(jcmdh),
    limits_(limits),
    prev_pos_(std::numeric_limits<double>::quiet_NaN()),
    prev_vel_(0.0)
  {
  }

  JointLimitHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jvelh,
    const hardware_interface::JointHandle & jcmdh, const JointLimits & limits)
  : jposh_(jposh),
    jvelh_(jvelh),
    jcmdh_(jcmdh),
    limits_(limits),
    prev_pos_(std::numeric_limits<double>::quiet_NaN()),
    prev_vel_(0.0)
  {
  }

  /// \return Joint name.
  std::string get_name() const
  {
    return jposh_   ? jposh_.get_name()
           : jvelh_ ? jvelh_.get_name()
           : jcmdh_ ? jcmdh_.get_name()
                    : std::string();
  }

  /// Sub-class implementation of limit enforcing policy.
  virtual void enforce_limits(const rclcpp::Duration & period) = 0;

  /// Clear stored state, causing it to reset next iteration.
  virtual void reset()
  {
    prev_pos_ = std::numeric_limits<double>::quiet_NaN();
    prev_vel_ = 0.0;
  }

protected:
  hardware_interface::JointHandle jposh_;
  hardware_interface::JointHandle jvelh_;
  hardware_interface::JointHandle jcmdh_;
  joint_limits_interface::JointLimits limits_;

  // stored state - track position and velocity of last update
  double prev_pos_;
  double prev_vel_;

  /// Return velocity for limit calculations.
  /**
   * @param period Time since last measurement
   * @return the velocity, from state if available, otherwise from previous position history.
   */
  double get_velocity(const rclcpp::Duration & period) const
  {
    // if we have a handle to a velocity state we can directly return state velocity
    // otherwise we will estimate velocity from previous position (command or state)
    return jvelh_ ? jvelh_.get_value() : (jposh_.get_value() - prev_pos_) / period.seconds();
  }
};

/** The base class of limit handles for enforcing position, velocity, and effort limits of
 * an effort-controlled joint that has soft-limits.
 */
class JointSoftLimitsHandle : public JointLimitHandle
{
public:
  JointSoftLimitsHandle() {}

  JointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const JointLimits & limits, const SoftJointLimits & soft_limits)
  : JointLimitHandle(jposh, jcmdh, limits), soft_limits_(soft_limits)
  {
  }

  JointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jvelh,
    const hardware_interface::JointHandle & jcmdh, const JointLimits & limits,
    const SoftJointLimits & soft_limits)
  : JointLimitHandle(jposh, jvelh, jcmdh, limits), soft_limits_(soft_limits)
  {
  }

protected:
  joint_limits_interface::SoftJointLimits soft_limits_;
};

/** A handle used to enforce position and velocity limits of a position-controlled joint that does
   not have soft limits. */
class PositionJointSaturationHandle : public JointLimitHandle
{
public:
  PositionJointSaturationHandle() {}

  PositionJointSaturationHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const JointLimits & limits)
  : JointLimitHandle(jposh, jcmdh, limits)
  {
    if (limits_.has_position_limits)
    {
      min_pos_limit_ = limits_.min_position;
      max_pos_limit_ = limits_.max_position;
    }
    else
    {
      min_pos_limit_ = -std::numeric_limits<double>::max();
      max_pos_limit_ = std::numeric_limits<double>::max();
    }
  }

  /// Enforce position and velocity limits for a joint that is not subject to soft limits.
  /**
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period)
  {
    if (std::isnan(prev_pos_))
    {
      prev_pos_ = jposh_.get_value();
    }

    double min_pos, max_pos;
    if (limits_.has_velocity_limits)
    {
      // enforce velocity limits
      // set constraints on where the position can be based on the
      // max velocity times seconds since last update
      const double delta_pos = limits_.max_velocity * period.seconds();
      min_pos = std::max(prev_pos_ - delta_pos, min_pos_limit_);
      max_pos = std::min(prev_pos_ + delta_pos, max_pos_limit_);
    }
    else
    {
      // no velocity limit, so position is simply limited to set extents (our imposed soft limits)
      min_pos = min_pos_limit_;
      max_pos = max_pos_limit_;
    }

    // clamp command position to our computed min/max position
    const double cmd = rcppmath::clamp(jcmdh_.get_value(), min_pos, max_pos);
    jcmdh_.set_value(cmd);

    prev_pos_ = cmd;
  }

private:
  double min_pos_limit_, max_pos_limit_;
};

/// A handle used to enforce position and velocity limits of a position-controlled joint.
/**
 * This class implements a very simple position and velocity limits enforcing policy, and tries to
 * impose the least amount of requisites on the underlying hardware platform. This lowers
 * considerably the entry barrier to use it, but also implies some limitations.
 *
 * <b>Requisites</b>
 * - Position (for non-continuous joints) and velocity limits specification.
 * - Soft limits specification. The \c k_velocity parameter is \e not used.
 *
 * <b>Open loop nature</b>
 *
 * Joint position and velocity limits are enforced in an open-loop fashion, that is, the command is
 * checked for validity without relying on the actual position/velocity values.
 *
 * - Actual position values are \e not used because in some platforms there might be a substantial
 * lag between sending a command and executing it (propagate command to hardware, reach control
 * objective, read from hardware).
 *
 * - Actual velocity values are \e not used because of the above reason, and because some platforms
 * might not expose trustworthy velocity measurements, or none at all.
 *
 * The downside of the open loop behavior is that velocity limits will not be enforced when
 * recovering from large position tracking errors. Only the command is guaranteed to comply with the
 * limits specification.
 *
 * \note: This handle type is \e stateful, ie. it stores the previous position command to estimate
 * the command velocity.
 */

// TODO(anyone): Leverage %Reflexxes Type II library for acceleration limits handling?
class PositionJointSoftLimitsHandle : public JointSoftLimitsHandle
{
public:
  PositionJointSoftLimitsHandle() {}

  PositionJointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits,
    const joint_limits_interface::SoftJointLimits & soft_limits)
  : JointSoftLimitsHandle(jposh, jcmdh, limits, soft_limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no velocity limits specification.");
    }
  }

  /// Enforce position and velocity limits for a joint subject to soft limits.
  /**
   * If the joint has no position limits (eg. a continuous joint), only velocity limits will be
   * enforced.
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period) override
  {
    assert(period.seconds() > 0.0);

    // Current position
    if (std::isnan(prev_pos_))
    {
      // Happens only once at initialization
      prev_pos_ = jposh_.get_value();
    }
    const double pos = prev_pos_;

    // Velocity bounds
    double soft_min_vel;
    double soft_max_vel;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.min_position), -limits_.max_velocity,
        limits_.max_velocity);

      soft_max_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.max_position), -limits_.max_velocity,
        limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel = limits_.max_velocity;
    }

    // Position bounds
    const double dt = period.seconds();
    double pos_low = pos + soft_min_vel * dt;
    double pos_high = pos + soft_max_vel * dt;

    if (limits_.has_position_limits)
    {
      // This extra measure safeguards against pathological cases, like when the soft limit lies
      // beyond the hard limit
      pos_low = std::max(pos_low, limits_.min_position);
      pos_high = std::min(pos_high, limits_.max_position);
    }

    // Saturate position command according to bounds
    const double pos_cmd = rcppmath::clamp(jcmdh_.get_value(), pos_low, pos_high);
    jcmdh_.set_value(pos_cmd);

    // Cache variables
    // todo: shouldn't this just be pos_cmd? why call into the command handle to
    //  get what we have in the above line?
    prev_pos_ = jcmdh_.get_value();
  }
};

/**
 * A handle used to enforce position, velocity, and effort limits of an effort-controlled
 * joint that does not have soft limits.
 */
class EffortJointSaturationHandle : public JointLimitHandle
{
public:
  EffortJointSaturationHandle() {}

  EffortJointSaturationHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jvelh,
    const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits)
  : JointLimitHandle(jposh, jvelh, jcmdh, limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no efforts limits specification.");
    }
  }

  EffortJointSaturationHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits)
  : EffortJointSaturationHandle(
      jposh, hardware_interface::JointHandle(hardware_interface::HW_IF_VELOCITY), jcmdh, limits)
  {
  }

  /**
   * Enforce position, velocity, and effort limits for a joint that is not subject
   * to soft limits.
   *
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period) override
  {
    double min_eff = -limits_.max_effort;
    double max_eff = limits_.max_effort;

    if (limits_.has_position_limits)
    {
      const double pos = jposh_.get_value();
      if (pos < limits_.min_position)
      {
        min_eff = 0.0;
      }
      else if (pos > limits_.max_position)
      {
        max_eff = 0.0;
      }
    }

    const double vel = get_velocity(period);
    if (vel < -limits_.max_velocity)
    {
      min_eff = 0.0;
    }
    else if (vel > limits_.max_velocity)
    {
      max_eff = 0.0;
    }

    double clamped = rcppmath::clamp(jcmdh_.get_value(), min_eff, max_eff);
    jcmdh_.set_value(clamped);
  }
};

/// A handle used to enforce position, velocity and effort limits of an effort-controlled joint.
// TODO(anyone): This class is untested!. Update unit tests accordingly.
class EffortJointSoftLimitsHandle : public JointSoftLimitsHandle
{
public:
  EffortJointSoftLimitsHandle() {}

  EffortJointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jvelh,
    const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits,
    const joint_limits_interface::SoftJointLimits & soft_limits)
  : JointSoftLimitsHandle(jposh, jvelh, jcmdh, limits, soft_limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no effort limits specification.");
    }
  }

  EffortJointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits,
    const joint_limits_interface::SoftJointLimits & soft_limits)
  : EffortJointSoftLimitsHandle(
      jposh, hardware_interface::JointHandle(hardware_interface::HW_IF_VELOCITY), jcmdh, limits,
      soft_limits)
  {
  }

  /// Enforce position, velocity and effort limits for a joint subject to soft limits.
  /**
   * If the joint has no position limits (eg. a continuous joint), only velocity and effort limits
   * will be enforced.
   *
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period) override
  {
    // Current state
    const double pos = jposh_.get_value();
    const double vel = get_velocity(period);

    // Velocity bounds
    double soft_min_vel;
    double soft_max_vel;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.min_position), -limits_.max_velocity,
        limits_.max_velocity);

      soft_max_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.max_position), -limits_.max_velocity,
        limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel = limits_.max_velocity;
    }

    // Effort bounds depend on the velocity and effort bounds
    const double soft_min_eff = rcppmath::clamp(
      -soft_limits_.k_velocity * (vel - soft_min_vel), -limits_.max_effort, limits_.max_effort);

    const double soft_max_eff = rcppmath::clamp(
      -soft_limits_.k_velocity * (vel - soft_max_vel), -limits_.max_effort, limits_.max_effort);

    // Saturate effort command according to bounds
    const double eff_cmd = rcppmath::clamp(jcmdh_.get_value(), soft_min_eff, soft_max_eff);
    jcmdh_.set_value(eff_cmd);
  }
};

/// A handle used to enforce velocity and acceleration limits of a velocity-controlled joint.
class VelocityJointSaturationHandle : public JointLimitHandle
{
public:
  VelocityJointSaturationHandle() {}

  VelocityJointSaturationHandle(
    const hardware_interface::JointHandle & jvelh,  // currently unused
    const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits)
  : JointLimitHandle(
      hardware_interface::JointHandle(hardware_interface::HW_IF_POSITION), jvelh, jcmdh, limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no velocity limits specification.");
    }
  }

  VelocityJointSaturationHandle(
    const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits)
  : JointLimitHandle(
      hardware_interface::JointHandle(hardware_interface::HW_IF_POSITION),
      hardware_interface::JointHandle(hardware_interface::HW_IF_VELOCITY), jcmdh, limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException(
        "Cannot enforce limits for joint '" + get_name() +
        "'. It has no velocity limits specification.");
    }
  }

  /// Enforce joint velocity and acceleration limits.
  /**
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period) override
  {
    // Velocity bounds
    double vel_low;
    double vel_high;

    if (limits_.has_acceleration_limits)
    {
      assert(period.seconds() > 0.0);
      const double dt = period.seconds();

      vel_low = std::max(prev_vel_ - limits_.max_acceleration * dt, -limits_.max_velocity);
      vel_high = std::min(prev_vel_ + limits_.max_acceleration * dt, limits_.max_velocity);
    }
    else
    {
      vel_low = -limits_.max_velocity;
      vel_high = limits_.max_velocity;
    }

    // Saturate velocity command according to limits
    const double vel_cmd = rcppmath::clamp(jcmdh_.get_value(), vel_low, vel_high);
    jcmdh_.set_value(vel_cmd);

    // Cache variables
    prev_vel_ = jcmdh_.get_value();
  }
};

/**
 * A handle used to enforce position, velocity, and acceleration limits of a
 * velocity-controlled joint.
 */
class VelocityJointSoftLimitsHandle : public JointSoftLimitsHandle
{
public:
  VelocityJointSoftLimitsHandle() {}

  VelocityJointSoftLimitsHandle(
    const hardware_interface::JointHandle & jposh, const hardware_interface::JointHandle & jvelh,
    const hardware_interface::JointHandle & jcmdh,
    const joint_limits_interface::JointLimits & limits,
    const joint_limits_interface::SoftJointLimits & soft_limits)
  : JointSoftLimitsHandle(jposh, jvelh, jcmdh, limits, soft_limits)
  {
    if (limits.has_velocity_limits)
    {
      max_vel_limit_ = limits.max_velocity;
    }
    else
    {
      max_vel_limit_ = std::numeric_limits<double>::max();
    }
  }

  /**
   * Enforce position, velocity, and acceleration limits for a velocity-controlled joint
   * subject to soft limits.
   *
   * \param[in] period Control period.
   */
  void enforce_limits(const rclcpp::Duration & period)
  {
    double min_vel, max_vel;
    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit.
      const double pos = jposh_.get_value();
      min_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.min_position), -max_vel_limit_,
        max_vel_limit_);
      max_vel = rcppmath::clamp(
        -soft_limits_.k_position * (pos - soft_limits_.max_position), -max_vel_limit_,
        max_vel_limit_);
    }
    else
    {
      min_vel = -max_vel_limit_;
      max_vel = max_vel_limit_;
    }

    if (limits_.has_acceleration_limits)
    {
      const double vel = get_velocity(period);
      const double delta_t = period.seconds();
      min_vel = std::max(vel - limits_.max_acceleration * delta_t, min_vel);
      max_vel = std::min(vel + limits_.max_acceleration * delta_t, max_vel);
    }

    jcmdh_.set_value(rcppmath::clamp(jcmdh_.get_value(), min_vel, max_vel));
  }

private:
  double max_vel_limit_;
};

// TODO(anyone): Port this to ROS 2
// //**
//  * Interface for enforcing joint limits.
//  *
//  * \tparam HandleType %Handle type. Must implement the following methods:
//  *  \code
//  *   void enforce_limits();
//  *   std::string get_name() const;
//  *  \endcode
//  */
// template<class HandleType>
// class joint_limits_interface::JointLimitsInterface
//   : public hardware_interface::ResourceManager<HandleType>
// {
// public:
//   HandleType getHandle(const std::string & name)
//   {
//     // Rethrow exception with a meaningful type
//     try {
//       return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
//     } catch (const std::logic_error & e) {
//       throw joint_limits_interface::JointLimitsInterfaceException(e.what());
//     }
//   }
//
//   /** \name Real-Time Safe Functions
//    *\{*/
//   /** Enforce limits for all managed handles. */
//   void enforce_limits(const rclcpp::Duration & period)
//   {
//     for (auto && resource_name_and_handle : this->resource_map_) {
//       resource_name_and_handle.second.enforce_limits(period);
//     }
//   }
//   /*\}*/
// };
//
// /** Interface for enforcing limits on a position-controlled joint through saturation. */
// class PositionJointSaturationInterface
//   : public joint_limits_interface::JointLimitsInterface<PositionJointSaturationHandle>
// {
// public:
//   /** \name Real-Time Safe Functions
//    *\{*/
//   /** Reset all managed handles. */
//   void reset()
//   {
//     for (auto && resource_name_and_handle : this->resource_map_) {
//       resource_name_and_handle.second.reset();
//     }
//   }
//   /*\}*/
// };
//
// /** Interface for enforcing limits on a position-controlled joint with soft position limits. */
// class PositionJointSoftLimitsInterface
//   : public joint_limits_interface::JointLimitsInterface<PositionJointSoftLimitsHandle>
// {
// public:
//   /** \name Real-Time Safe Functions
//    *\{*/
//   /** Reset all managed handles. */
//   void reset()
//   {
//     for (auto && resource_name_and_handle : this->resource_map_) {
//       resource_name_and_handle.second.reset();
//     }
//   }
//   /*\}*/
// };
//
// /** Interface for enforcing limits on an effort-controlled joint through saturation. */
// class EffortJointSaturationInterface
//   : public joint_limits_interface::JointLimitsInterface<EffortJointSaturationHandle>
// {
// };
//
// /** Interface for enforcing limits on an effort-controlled joint with soft position limits. */
// class EffortJointSoftLimitsInterface
//   : public joint_limits_interface::JointLimitsInterface<EffortJointSoftLimitsHandle>
// {
// };
//
// /** Interface for enforcing limits on a velocity-controlled joint through saturation. */
// class VelocityJointSaturationInterface
//   : public joint_limits_interface::JointLimitsInterface<VelocityJointSaturationHandle>
// {
// };
//
// /** Interface for enforcing limits on a velocity-controlled joint with soft position limits. */
// class VelocityJointSoftLimitsInterface
//   : public joint_limits_interface::JointLimitsInterface<VelocityJointSoftLimitsHandle>
// {
// };
}  // namespace joint_limits_interface

#endif  // JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_HPP_
