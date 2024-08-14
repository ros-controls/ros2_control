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
#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "transmission_interface/handle.hpp"

namespace transmission_interface
{
/// Abstract base class for representing mechanical transmissions.
/**
 * Mechanical transmissions transform effort/flow variables such that their product (power) remains
 * constant. Effort variables for linear and rotational domains are \e force and \e torque; while
 * the flow variables are respectively linear velocity and angular velocity.
 *
 * In robotics it is customary to place transmissions between actuators and joints. This interface
 * adheres to this naming to identify the input and output spaces of the transformation. The
 * provided interfaces allow bidirectional mappings between actuator and joint spaces for effort,
 * velocity and position. Position is not a power variable, but the mappings can be implemented
 * using the velocity map plus an integration constant representing the offset between actuator and
 * joint zeros.
 *
 * \par Credit
 * This interface was inspired by similar existing implementations by PAL Robotics, S.L. and Willow
 * Garage Inc.
 *
 * \note Implementations of this interface must take care of realtime-safety if the code is to be
 * run in realtime contexts, as is often the case in robot control.
 */
class Transmission
{
public:
  virtual ~Transmission() = default;

  virtual void configure(
    const std::vector<JointHandle> & joint_handles,
    const std::vector<ActuatorHandle> & actuator_handles) = 0;

  /// Transform \e effort variables from actuator to joint space.
  /**
   * \param[in] act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the
   * number of transmission actuators and joints. Data vectors not used in this map can remain
   * empty.
   */
  virtual void actuator_to_joint() = 0;

  /// Transform \e effort variables from joint to actuator space.
  /**
   * \param[in] jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the
   * number of transmission actuators and joints. Data vectors not used in this map can remain
   * empty.
   */
  virtual void joint_to_actuator() = 0;

  /** \return Number of actuators managed by transmission,
   *  ie. the dimension of the actuator space.
   */
  virtual std::size_t num_actuators() const = 0;

  /** \return Number of joints managed by transmission,
   * ie. the dimension of the joint space.
   */
  virtual std::size_t num_joints() const = 0;
};

typedef std::shared_ptr<Transmission> TransmissionSharedPtr;

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_HPP_
