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

#ifndef TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_HPP_

#include <cassert>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/accessor.hpp"
#include "transmission_interface/exception.hpp"
#include "transmission_interface/transmission.hpp"

namespace transmission_interface
{
/// Implementation of a four-bar-linkage transmission.
/**
 *
 * This transmission relates <b>two actuators</b> and <b>two joints</b> through a mechanism in which
 * the state of the first joint only depends on the first actuator, while the second joint depends
 * on both actuators, as illustrated below. Although the class name makes specific reference to the
 * four-bar-linkage, there are other mechanical layouts that yield the same behavior, such as the
 * remote actuation example also depicted below. \image html four_bar_linkage_transmission.png
 *
 * <CENTER>
 * <table>
 * <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
 * <tr><td>
 * <b> Actuator to joint </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{j_1} & = & n_{j_1} n_{a_1} \tau_{a_1} \\
 * \tau_{j_2} & = & n_{j_2} (n_{a_2} \tau_{a_2} - n_{j_1} n_{a_1} \tau_{a_1})
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{j_1} & = & \frac{ \dot{x}_{a_1} }{ n_{j_1} n_{a_1} } \\
 * \dot{x}_{j_2} & = & \frac{ \dot{x}_{a_2} / n_{a_2} - \dot{x}_{a_1} / (n_{j_1} n_{a_1}) }{ n_{j_2}
 * } \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{j_1} & = & \frac{ x_{a_1} }{ n_{j_1} n_{a_1} } + x_{off_1} \\
 * x_{j_2} & = & \frac{ x_{a_2} / n_{a_2} - x_{a_1} / (n_{j_1} n_{a_1}) }{ n_{j_2} } + x_{off_2}
 * \f}
 * </td>
 * </tr>
 * <tr><td>
 * <b> Joint to actuator </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{a_1} & = & \tau_{j_1} / (n_{j_1} n_{a_1}) \\
 * \tau_{a_2} & = & \frac{ \tau_{j_1} + \tau_{j_2} / n_{j_2} }{ n_{a_2} }
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{a_1} & = & n_{j_1} n_{a_1} \dot{x}_{j_1} \\
 * \dot{x}_{a_2} & = & n_{a_2} (\dot{x}_{j_1} + n_{j_2} \dot{x}_{j_2})
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{a_1} & = & n_{j_1} n_{a_1} (x_{j_1} - x_{off_1}) \\
 * x_{a_2} & = & n_{a_2} \left[(x_{j_1} - x_{off_1}) + n_{j_2} (x_{j_2}  - x_{off_2})\right]
 * \f}
 * </td></tr></table>
 * </CENTER>
 *
 * where:
 * - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables,
 * respectively.
 * - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space
 * variables, respectively.
 * - \f$ x_{off}\f$ represents the offset between motor and joint zeros, expressed in joint position
 * coordinates. (cf. SimpleTransmission class documentation for a more detailed description of this
 * variable).
 * - \f$ n \f$ represents a transmission ratio (reducers/amplifiers are depicted as timing belts in
 * the figure). A transmission ratio can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer / effort amplifier,
 * while if its absolute value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
 *     - Negative values represent a direction flip, ie. input and output move in opposite
 * directions.
 *     - <b>Important:</b> Use transmission ratio signs to match this class' convention of positive
 * actuator/joint directions with a given mechanical design, as they will in general not match.
 *
 * \ingroup transmission_types
 */
class FourBarLinkageTransmission : public Transmission
{
public:
  /**
   * \param actuator_reduction Reduction ratio of actuators.
   * \param joint_reduction    Reduction ratio of joints.
   * \param joint_offset       Joint position offset used in the position mappings.
   * \pre Nonzero actuator reduction values.
   */
  FourBarLinkageTransmission(
    const std::vector<double> & actuator_reduction, const std::vector<double> & joint_reduction,
    const std::vector<double> & joint_offset = std::vector<double>(2, 0.0));

  /**
   * \param[in] joint_handles     Handles of joint values.
   * \param[in] actuator_handles  Handles of actuator values.
   * \pre Handles are valid and matching in size
   */
  void configure(
    const std::vector<JointHandle> & joint_handles,
    const std::vector<ActuatorHandle> & actuator_handles) override;

  /// Transform variables from actuator to joint space.
  /**
   * \pre Actuator and joint vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can
   * even remain empty.
   */
  void actuator_to_joint() override;

  /// Transform variables from joint to actuator space.
  /**
   * \pre Actuator and joint vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can
   * even remain empty.
   */
  void joint_to_actuator() override;

  std::size_t num_actuators() const override { return 2; }
  std::size_t num_joints() const override { return 2; }

  const std::vector<double> & get_actuator_reduction() const { return actuator_reduction_; }
  const std::vector<double> & get_joint_reduction() const { return joint_reduction_; }
  const std::vector<double> & get_joint_offset() const { return joint_offset_; }

  /// Get human-friendly report of handles
  std::string get_handles_info() const;

protected:
  std::vector<double> actuator_reduction_;
  std::vector<double> joint_reduction_;
  std::vector<double> joint_offset_;

  std::vector<JointHandle> joint_position_;
  std::vector<JointHandle> joint_velocity_;
  std::vector<JointHandle> joint_effort_;

  std::vector<ActuatorHandle> actuator_position_;
  std::vector<ActuatorHandle> actuator_velocity_;
  std::vector<ActuatorHandle> actuator_effort_;
};

inline FourBarLinkageTransmission::FourBarLinkageTransmission(
  const std::vector<double> & actuator_reduction, const std::vector<double> & joint_reduction,
  const std::vector<double> & joint_offset)
: actuator_reduction_(actuator_reduction),
  joint_reduction_(joint_reduction),
  joint_offset_(joint_offset)
{
  if (
    num_actuators() != actuator_reduction_.size() || num_joints() != joint_reduction_.size() ||
    num_joints() != joint_offset_.size())
  {
    throw Exception("Reduction and offset vectors must have size 2.");
  }
  if (
    0.0 == actuator_reduction_[0] || 0.0 == actuator_reduction_[1] || 0.0 == joint_reduction_[0] ||
    0.0 == joint_reduction_[1])
  {
    throw Exception("Transmission reduction ratios cannot be zero.");
  }
}

void FourBarLinkageTransmission::configure(
  const std::vector<JointHandle> & joint_handles,
  const std::vector<ActuatorHandle> & actuator_handles)
{
  if (joint_handles.empty())
  {
    throw Exception("No joint handles were passed in");
  }

  if (actuator_handles.empty())
  {
    throw Exception("No actuator handles were passed in");
  }

  const auto joint_names = get_names(joint_handles);
  if (joint_names.size() != 2)
  {
    throw Exception(
      "There should be exactly two unique joint names but was given " + to_string(joint_names));
  }
  const auto actuator_names = get_names(actuator_handles);
  if (actuator_names.size() != 2)
  {
    throw Exception(
      "There should be exactly two unique actuator names but was given " +
      to_string(actuator_names));
  }

  joint_position_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_POSITION);
  joint_velocity_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_VELOCITY);
  joint_effort_ = get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_EFFORT);

  if (joint_position_.size() != 2 && joint_velocity_.size() != 2 && joint_effort_.size() != 2)
  {
    throw Exception("Not enough valid or required joint handles were presented.");
  }

  actuator_position_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_POSITION);
  actuator_velocity_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_VELOCITY);
  actuator_effort_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_EFFORT);

  if (
    actuator_position_.size() != 2 && actuator_velocity_.size() != 2 &&
    actuator_effort_.size() != 2)
  {
    throw Exception(
      "Not enough valid or required actuator handles were presented. \n" + get_handles_info());
  }

  if (
    joint_position_.size() != actuator_position_.size() &&
    joint_velocity_.size() != actuator_velocity_.size() &&
    joint_effort_.size() != actuator_effort_.size())
  {
    throw Exception("Pair-wise mismatch on interfaces. \n" + get_handles_info());
  }
}

inline void FourBarLinkageTransmission::actuator_to_joint()
{
  const auto & ar = actuator_reduction_;
  const auto & jr = joint_reduction_;

  // position
  auto & act_pos = actuator_position_;
  auto & joint_pos = joint_position_;
  if (act_pos.size() == num_actuators() && joint_pos.size() == num_joints())
  {
    assert(act_pos[0] && act_pos[1] && joint_pos[0] && joint_pos[1]);

    joint_pos[0].set_value(act_pos[0].get_value() / (jr[0] * ar[0]) + joint_offset_[0]);
    joint_pos[1].set_value(
      (act_pos[1].get_value() / ar[1] - act_pos[0].get_value() / (jr[0] * ar[0])) / jr[1] +
      joint_offset_[1]);
  }

  // velocity
  auto & act_vel = actuator_velocity_;
  auto & joint_vel = joint_velocity_;
  if (act_vel.size() == num_actuators() && joint_vel.size() == num_joints())
  {
    assert(act_vel[0] && act_vel[1] && joint_vel[0] && joint_vel[1]);

    joint_vel[0].set_value(act_vel[0].get_value() / (jr[0] * ar[0]));
    joint_vel[1].set_value(
      (act_vel[1].get_value() / ar[1] - act_vel[0].get_value() / (jr[0] * ar[0])) / jr[1]);
  }

  // effort
  auto & act_eff = actuator_effort_;
  auto & joint_eff = joint_effort_;
  if (act_eff.size() == num_actuators() && joint_eff.size() == num_joints())
  {
    assert(act_eff[0] && act_eff[1] && joint_eff[0] && joint_eff[1]);

    joint_eff[0].set_value(jr[0] * act_eff[0].get_value() * ar[0]);
    joint_eff[1].set_value(
      jr[1] * (act_eff[1].get_value() * ar[1] - jr[0] * act_eff[0].get_value() * ar[0]));
  }
}

inline void FourBarLinkageTransmission::joint_to_actuator()
{
  const auto & ar = actuator_reduction_;
  const auto & jr = joint_reduction_;

  // position
  auto & act_pos = actuator_position_;
  auto & joint_pos = joint_position_;
  if (act_pos.size() == num_actuators() && joint_pos.size() == num_joints())
  {
    assert(act_pos[0] && act_pos[1] && joint_pos[0] && joint_pos[1]);

    double joints_offset_applied[2] = {
      joint_pos[0].get_value() - joint_offset_[0], joint_pos[1].get_value() - joint_offset_[1]};
    act_pos[0].set_value(joints_offset_applied[0] * jr[0] * ar[0]);
    act_pos[1].set_value((joints_offset_applied[0] + joints_offset_applied[1] * jr[1]) * ar[1]);
  }

  // velocity
  auto & act_vel = actuator_velocity_;
  auto & joint_vel = joint_velocity_;
  if (act_vel.size() == num_actuators() && joint_vel.size() == num_joints())
  {
    assert(act_vel[0] && act_vel[1] && joint_vel[0] && joint_vel[1]);

    act_vel[0].set_value(joint_vel[0].get_value() * jr[0] * ar[0]);
    act_vel[1].set_value((joint_vel[0].get_value() + joint_vel[1].get_value() * jr[1]) * ar[1]);
  }

  // effort
  auto & act_eff = actuator_effort_;
  auto & joint_eff = joint_effort_;
  if (act_eff.size() == num_actuators() && joint_eff.size() == num_joints())
  {
    assert(act_eff[0] && act_eff[1] && joint_eff[0] && joint_eff[1]);

    act_eff[0].set_value(joint_eff[0].get_value() / (ar[0] * jr[0]));
    act_eff[1].set_value((joint_eff[0].get_value() + joint_eff[1].get_value() / jr[1]) / ar[1]);
  }
}

std::string FourBarLinkageTransmission::get_handles_info() const
{
  return std::string("Got the following handles:\n") +
         "Joint position: " + to_string(get_names(joint_position_)) +
         ", Actuator position: " + to_string(get_names(actuator_position_)) + "\n" +
         "Joint velocity: " + to_string(get_names(joint_velocity_)) +
         ", Actuator velocity: " + to_string(get_names(actuator_velocity_)) + "\n" +
         "Joint effort: " + to_string(get_names(joint_effort_)) +
         ", Actuator effort: " + to_string(get_names(actuator_effort_));
}

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_HPP_
