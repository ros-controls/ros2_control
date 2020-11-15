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
#ifndef TRANSMISSION_INTERFACE__SIMPLE_TRANSMISSION_H_
#define TRANSMISSION_INTERFACE__SIMPLE_TRANSMISSION_H_

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "transmission_interface/transmission.h"
#include "transmission_interface/transmission_interface_exception.h"

namespace transmission_interface
{

/**
 * \brief Implementation of a simple reducer transmission.
 *
 * This transmission relates <b>one actuator</b> and <b>one joint</b> through a reductor (or amplifier).
 * Timing belts and gears are examples of this transmission type, and are illustrated below.
 * \image html simple_transmission.png
 *
 * <CENTER>
 * <table>
 * <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
 * <tr><td>
 * <b> Actuator to joint </b>
 * </td>
 * <td>
 * \f[ \tau_j = n \tau_a \f]
 * </td>
 * <td>
 * \f[ \dot{x}_j = \dot{x}_a / n \f]
 * </td>
 * <td>
 * \f[ x_j = x_a / n + x_{off} \f]
 * </td>
 * </tr>
 * <tr><td>
 * <b> Joint to actuator </b>
 * </td>
 * <td>
 * \f[ \tau_a = \tau_j / n\f]
 * </td>
 * <td>
 * \f[ \dot{x}_a = n \dot{x}_j \f]
 * </td>
 * <td>
 * \f[ x_a = n (x_j - x_{off}) \f]
 * </td></tr></table>
 * </CENTER>
 *
 * where:
 * - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables, respectively.
 * - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space variables, respectively.
 * - \f$ x_{off}\f$ represents the offset between motor and joint zeros, expressed in joint position coordinates.
 * - \f$ n \f$ is the transmission ratio, and can be computed as the ratio between the output and input pulley
 *   radii for the timing belt; or the ratio between output and input teeth for the gear system.
 *   The transmission ratio can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer / effort amplifier, while if its absolute
 *       value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
 *     - Negative values represent a direction flip, ie. actuator and joint move in opposite directions. For example,
 *       in timing belts actuator and joint move in the same direction, while in single-stage gear systems actuator and
 *       joint move in opposite directions.
 *
 * \ingroup transmission_types
 */
class SimpleTransmission: public Transmission
{
public:
    /**
     * \param reduction Reduction ratio.
     * \param joint_offset Joint position offset used in the position mappings.
     * \pre Nonzero reduction value.
     */
    SimpleTransmission(
      const double reduction,
      const double joint_offset = 0.0);

    void configure(
      const std::vector < hardware_interface::CommandInterface > & joint_handles,
      const std::vector < hardware_interface::CommandInterface > & actuator_handles) override;

    /**
     * \brief Transform \e effort variables from actuator to joint space.
     * \param[in]  act_data Actuator-space variables.
     * \param[out] jnt_data Joint-space variables.
     * \pre Actuator and joint effort vectors must have size 1 and point to valid data.
     *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
     */
    void actuator_to_joint() override;

    /**
     * \brief Transform \e effort variables from joint to actuator space.
     * \param[in]  jnt_data Joint-space variables.
     * \param[out] act_data Actuator-space variables.
     * \pre Actuator and joint effort vectors must have size 1 and point to valid data.
     *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
     */
    void joint_to_actuator() override;

    std::size_t num_actuators() const override {return 1;}
    std::size_t num_joints()    const override {return 1;}

    double get_actuator_reduction() const {return reduction_;}
    double get_joint_offset()       const {return jnt_offset_;}

protected:
    double reduction_;
    double jnt_offset_;

    hardware_interface::CommandInterface joint_position = {"", "", nullptr};
    hardware_interface::CommandInterface joint_velocity = {"", "", nullptr};
    hardware_interface::CommandInterface joint_effort = {"", "", nullptr};

    hardware_interface::CommandInterface actuator_position = {"", "", nullptr};
    hardware_interface::CommandInterface actuator_velocity = {"", "", nullptr};
    hardware_interface::CommandInterface actuator_effort = {"", "", nullptr};
  };

  inline SimpleTransmission::SimpleTransmission(
    const double reduction,
    const double joint_offset)
  : reduction_(reduction),
    jnt_offset_(joint_offset)
  {
    if (0.0 == reduction_) {
      throw Exception("Transmission reduction ratio cannot be zero.");
    }
  }

  hardware_interface::CommandInterface get_by_interface(
    const std::vector < hardware_interface::CommandInterface > & handles,
    const std::string & interface_name)
  {
    const auto result =
      std::find_if(
      handles.cbegin(), handles.cend(), [&interface_name](
        const auto handle) {return handle.get_interface_name() == interface_name;});
    if (result == handles.cend()) {
      return hardware_interface::CommandInterface("", "", nullptr);
    }
    return *result;
  }

  void SimpleTransmission::configure(
    const std::vector < hardware_interface::CommandInterface > & joint_handles,
    const std::vector < hardware_interface::CommandInterface > & actuator_handles)
  {
    // check that joint / act names are identical
    // check that joint interfaces and actuator interfaces match

    joint_position = get_by_interface(joint_handles, "position");
    joint_velocity = get_by_interface(joint_handles, "velocity");
    joint_effort = get_by_interface(joint_handles, "effort");

    actuator_position = get_by_interface(actuator_handles, "position");
    actuator_velocity = get_by_interface(actuator_handles, "velocity");
    actuator_effort = get_by_interface(actuator_handles, "effort");
  }

  inline void SimpleTransmission::actuator_to_joint()
  {
    if (joint_effort && actuator_effort) {
      joint_effort.set_value(actuator_effort.get_value() * reduction_);
    }

    if (joint_velocity && actuator_velocity) {
      joint_velocity.set_value(actuator_velocity.get_value() / reduction_);
    }

    if (joint_position && actuator_position) {
      joint_position.set_value(actuator_position.get_value() / reduction_ + jnt_offset_);
    }

//   *jnt_data.absolute_position[0] = *act_data.absolute_position[0] / reduction_ + jnt_offset_;
//   *jnt_data.torque_sensor[0] = *act_data.torque_sensor[0] * reduction_;
  }


  inline void SimpleTransmission::joint_to_actuator()
  {
    if (joint_effort && actuator_effort) {
      actuator_effort.set_value(joint_effort.get_value() / reduction_);
    }

    if (joint_velocity && actuator_velocity) {
      actuator_velocity.set_value(joint_velocity.get_value() * reduction_);
    }

    if (joint_position && actuator_position) {
      actuator_position.set_value((joint_position.get_value() - jnt_offset_) * reduction_);
    }
  }

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__SIMPLE_TRANSMISSION_H_
