// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components
{
constexpr std::size_t FORCES_SIZE = 3;
class ForceTorqueSensor : public SemanticComponentInterface<geometry_msgs::msg::Wrench>
{
public:
  /// Constructor for "standard" 6D FTS
  explicit ForceTorqueSensor(const std::string & name)
  : SemanticComponentInterface(
      name,
      // If 6D FTS use standard names
      {{name + "/" + "force.x"},
       {name + "/" + "force.y"},
       {name + "/" + "force.z"},
       {name + "/" + "torque.x"},
       {name + "/" + "torque.y"},
       {name + "/" + "torque.z"}}),
    existing_axes_({{true, true, true, true, true, true}})
  {
  }

  /// Constructor for 6D FTS with custom interface names.
  /**
   * Constructor for 6D FTS with custom interface names or FTS with less then six measurement axes,
   * e.g., 1D and 2D force load cells.
   * For non existing axes interface is empty string, i.e., ("");
   *
   * The name should be in the following order:
   *   force X, force Y, force Z, torque X, torque Y, torque Z.
   */
  ForceTorqueSensor(
    const std::string & interface_force_x, const std::string & interface_force_y,
    const std::string & interface_force_z, const std::string & interface_torque_x,
    const std::string & interface_torque_y, const std::string & interface_torque_z)
  : SemanticComponentInterface("", 6)
  {
    auto check_and_add_interface =
      [this](const std::string & interface_name, const std::size_t index)
    {
      if (!interface_name.empty())
      {
        interface_names_.emplace_back(interface_name);
        existing_axes_[index] = true;
      }
      else
      {
        existing_axes_[index] = false;
      }
    };

    check_and_add_interface(interface_force_x, 0);
    check_and_add_interface(interface_force_y, 1);
    check_and_add_interface(interface_force_z, 2);
    check_and_add_interface(interface_torque_x, 3);
    check_and_add_interface(interface_torque_y, 4);
    check_and_add_interface(interface_torque_z, 5);
  }

  /// Return forces.
  /**
   * Return forces of a FTS.
   *
   * \return array of size 3 with force values (x, y, z).
   */
  std::array<double, 3> get_forces() const
  {
    std::array<double, 3> forces;
    forces.fill(std::numeric_limits<double>::quiet_NaN());
    std::size_t interface_counter{0};
    for (auto i = 0u; i < forces.size(); ++i)
    {
      if (existing_axes_[i])
      {
        forces[i] = state_interfaces_[interface_counter].get().get_value();
        ++interface_counter;
      }
    }
    return forces;
  }

  /// Return torque.
  /**
   * Return torques of a FTS.
   *
   * \return array of size 3 with torque values (x, y, z).
   */
  std::array<double, 3> get_torques() const
  {
    std::array<double, 3> torques;
    torques.fill(std::numeric_limits<double>::quiet_NaN());

    // find out how many force interfaces are being used
    // torque interfaces will be found from the next index onward
    auto torque_interface_counter = static_cast<std::size_t>(
      std::count(existing_axes_.begin(), existing_axes_.begin() + FORCES_SIZE, true));

    for (auto i = 0u; i < torques.size(); ++i)
    {
      if (existing_axes_[i + FORCES_SIZE])
      {
        torques[i] = state_interfaces_[torque_interface_counter].get().get_value();
        ++torque_interface_counter;
      }
    }
    return torques;
  }

  /// Return Wrench message with forces and torques.
  /**
   * Constructs and return a wrench message from the current values.
   * The method assumes that the interface names on the construction are in the following order:
   *   force X, force Y, force Z, torque X, torque Y, torque Z.
   *
   * \return wrench message from values;
   */
  bool get_values_as_message(geometry_msgs::msg::Wrench & message) const
  {
    const auto [force_x, force_y, force_z] = get_forces();
    const auto [torque_x, torque_y, torque_z] = get_torques();

    message.force.x = force_x;
    message.force.y = force_y;
    message.force.z = force_z;
    message.torque.x = torque_x;
    message.torque.y = torque_y;
    message.torque.z = torque_z;

    return true;
  }

protected:
  /// Vector with existing axes for sensors with less then 6D axes.
  std::array<bool, 6> existing_axes_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_
