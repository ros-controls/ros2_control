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
/**
 * @brief Abstract base class for representing mechanical transmissions.
 *
 * Mechanical transmissions transform effort/flow variables such that their product (power) remains
 * constant. In the context of robotics, effort variables for linear and rotational domains are 
 * \e force and \e torque, while flow variables are \e linear velocity and \e angular velocity.
 *
 * Typically, transmissions are used to connect actuators and joints in robotic systems. This interface 
 * is designed to identify the input and output spaces of the transformation, offering bidirectional 
 * mappings between actuator and joint spaces for effort, velocity, and position. While position itself 
 * is not a power variable, mappings for position can be derived by using the velocity map and an 
 * integration constant to represent the offset between actuator and joint zeros.
 *
 * @par Credit
 * This interface was inspired by similar implementations by PAL Robotics, S.L., and Willow Garage Inc.
 *
 * @note Implementations of this interface must ensure realtime safety if the code is intended to 
 *       operate in real-time contexts, which is often the case in robot control applications.
 */

class Transmission
{
public:
  virtual ~Transmission() = default;

  virtual void configure(
    const std::vector<JointHandle> & joint_handles,
    const std::vector<ActuatorHandle> & actuator_handles) = 0;

 /**
 * @brief Transform \e effort variables from actuator to joint space.
 *
 * This method transforms effort variables, such as torque or force, from actuator-space to joint-space.
 *
 * @param[in] act_data Actuator-space variables.
 * @param[out] jnt_data Joint-space variables.
 *
 * @pre All non-empty vectors must contain valid data, and their sizes should be consistent with 
 *      the number of transmission actuators and joints. Data vectors not used in this map can remain empty.
 */

  virtual void actuator_to_joint() = 0;

 /**
 * @brief Transform \e effort variables from joint to actuator space.
 *
 * This method transforms effort variables, such as torque or force, from joint-space to actuator-space.
 *
 * @param[in] jnt_data Joint-space variables.
 * @param[out] act_data Actuator-space variables.
 *
 * @pre All non-empty vectors must contain valid data, and their sizes should be consistent with 
 *      the number of transmission actuators and joints. Data vectors not used in this map can remain empty.
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
