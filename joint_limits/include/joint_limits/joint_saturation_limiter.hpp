// Copyright (c) 2024, PickNik Inc.
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

/// \author Dr. Denis Stogl

#ifndef JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_
#define JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_

#include <memory>
#include <vector>

#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace joint_limits
{
/**
 * Joint Saturation Limiter limits joints' position, velocity and acceleration by clamping values
 * to its minimal and maximal allowed values. Since the position, velocity and accelerations are
 * variables in physical relation, it might be that some values are limited lower then specified
 * limit. For example, if a joint is close to its position limit, velocity and acceleration will be
 * reduced accordingly.
 */
template <typename JointLimitsStateDataType>
class JointSaturationLimiter : public JointLimiterInterface<JointLimitsStateDataType>
{
public:
  /** \brief Constructor */
  JointSaturationLimiter();

  /** \brief Destructor */
  virtual ~JointSaturationLimiter();

  bool on_init() override;

  bool on_configure(const JointLimitsStateDataType & current_joint_states) override
  {
    prev_command_ = current_joint_states;
    const size_t num_joints = this->number_of_joints_;

    desired_pos_.assign(num_joints, 0.0);
    desired_vel_.assign(num_joints, 0.0);
    desired_acc_.assign(num_joints, 0.0);
    expected_pos_.assign(num_joints, 0.0);
    expected_vel_.assign(num_joints, 0.0);

    pos_limit_hit_.assign(num_joints, false);
    vel_limit_hit_.assign(num_joints, false);
    acc_limit_hit_.assign(num_joints, false);
    dec_limit_hit_.assign(num_joints, false);
    jerk_limit_hit_.assign(num_joints, false);
    return true;
  }

  /** \brief Enforce joint limits to desired position, velocity and acceleration using clamping.
   * Class implements this method accepting following data types:
   * - trajectory_msgs::msg::JointTrajectoryPoint: limiting position, velocity and acceleration;
   *
   * Implementation of saturation approach for joints with position, velocity or acceleration limits
   * and values. First, position limits are checked to adjust desired velocity accordingly, then
   * velocity and finally acceleration.
   * The method support partial existence of limits, e.g., missing position limits for continuous
   * joins.
   * 
   * Steps:
   * 1. clamp joint limits -> update braking_near_position_limit_triggered
   * 2. if braking_near_position_limit_triggered -> handle_braking_near_position_limit
   * 3. update desired_joint_states for position, velocity, accleration
   *
   * \param[in] current_joint_states current joint states a robot is in.
   * \param[in,out] desired_joint_states joint state that should be adjusted to obey the limits.
   * \param[in] dt time delta to calculate missing integrals and derivation in joint limits.
   * \returns true if limits are enforced, otherwise false.
   * \throws std::runtime_error if the actual position is out of bounds if commanding position
   */
  bool on_enforce(
    const JointLimitsStateDataType & current_joint_states,
    JointLimitsStateDataType & desired_joint_states, const rclcpp::Duration & dt) override;

  /** \brief Reset internal states of the limiter. */
  /**
   * This method is called when the controller is stopped or when the controller is
   * reconfigured. It should reset all internal states of the limiter.
   */
  void reset_internals() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prev_command_ = JointLimitsStateDataType();
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  JointLimitsStateDataType prev_command_;
  std::mutex mutex_;

private:
  // Cached vectors to eliminate dynamic memory allocation (malloc) in the real-time execution loop
  std::vector<double> desired_pos_;
  std::vector<double> desired_vel_;
  std::vector<double> desired_acc_;
  std::vector<double> expected_vel_;
  std::vector<double> expected_pos_;

  // Pre-allocation boolean flags for tracking limits
  std::vector<bool> pos_limit_hit_;
  std::vector<bool> vel_limit_hit_;
  std::vector<bool> acc_limit_hit_;
  std::vector<bool> dec_limit_hit_;
  std::vector<bool> jerk_limit_hit_;

  /**
   * @brief
   * Clamps the joint limits
   * 
   * Steps:
   * 1. if has_desired_position -> has_position_limits -> clamp pos -> update desired_vel_
   * 2. has_velocity_limits -> if desired_vel not defined -> calc from (desired_pos - current_position) . dt ->
   * clamp vel -> if has_desired_position -> recalculate desired_pos -> calc desired_acc from vel
   * 3. if has_acc_limits || has_dec_limits -> if desired_acc not defined -> cal from (desired_vel - current_vel) / dt
   * -> if decelerating -> limit deceleration -> if accelerating -> limit acceleration -> if limit_applied -> 
   * cal desired_vel from current_joint_vel + desried_acceleration . dt -> if has_desired_positon -> calc desired_pos
   * 
   * if has_position_limits -> if has_desired_velosity && !has_desired_position -> calc expected_pos -> clamp pos -> if pos != expected_pos -> expected_pos = pos -> calc desired_vel
   * 
   * stopping_deccel = desired_vel / dt -> update stopping_deccel based on max_dec or mac_acc -> cal stopping_distance -> cal stopping_duration -> check if joint limits are beyond stopping_dist and desired_velocity -> else distance_after_stopping_duration -> recheck if jont limits are beyond -> update the bracking_near_pos_limit_trigger
   * 
   */
  void clamp_joint_limits(const bool has_desired_position, const bool has_desired_velocity, const bool has_desired_acceleration, const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states, trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, bool & limits_enforced, const std::vector<double> current_joint_velocities, bool & braking_near_position_limit_triggered,  const double dt_seconds);

  /**
   * @brief
   * Handles the braking near position limit
   * 
   * Steps:
   * for every joint -> cal desired_acc -> limit deceleration -> limit acceleration 
   * 
   * if has_desired_velocity -> update desired_vel = current_joint_velocity + desired_acc . dt 
   * 
   * if has_desired_position -> updated desired_pos = current_joint_position + current_joint_velocities . dt + 0.5 * desired_acc . dt . dt
   * 
   */
  void handle_braking_near_position_limit(
    const std::vector<double> & current_joint_velocities, double dt_seconds,
    bool has_desired_position, bool has_desired_velocity,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states);

};

template <typename JointLimitsStateDataType>
JointSaturationLimiter<JointLimitsStateDataType>::JointSaturationLimiter()
: JointLimiterInterface<JointLimitsStateDataType>()
{
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME));
}

template <typename JointLimitsStateDataType>
JointSaturationLimiter<JointLimitsStateDataType>::~JointSaturationLimiter()
{
}

template <typename JointLimitsStateDataType>
bool JointSaturationLimiter<JointLimitsStateDataType>::on_init()
{
  return true;
}

template <>
bool JointSaturationLimiter<JointControlInterfacesData>::on_init();

template <>
void JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>::clamp_joint_limits(const bool has_desired_position, const bool has_desired_velocity, const bool has_desired_acceleration, const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states, trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, bool & limits_enforced, const std::vector<double> current_joint_velocities, bool & braking_near_position_limit_triggered,  const double dt_seconds);

template <>
void JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>::handle_braking_near_position_limit(
  const std::vector<double> & current_joint_velocities, double dt_seconds,
  bool has_desired_position, bool has_desired_velocity,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states);

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_
