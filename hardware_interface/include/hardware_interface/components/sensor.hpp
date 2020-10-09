// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
{

using Deleter = std::function<void ()>;

/// Sensor wrapper class
/**
 * The class represents a wrapper around an implementation of the SensorInterface.
 * This allows to represent various implementations under the same common API.
 * Sensor serves as a handle which can get claimed and loaned to each individual controller
 * to obtain exclusive ownership over the component.
 */
class Sensor final
{
public:
  /// Constructor for a sensor component
  /**
   * The sensor is constructed with a precise SensorInterface implementation.
   * The second parameter serves as a callback upon destruction, which is used
   * to cleanup the component or signal its destruction to other entities such
   * as the resource manager within the controller manager.
   *
   * \param impl Shared pointer to a precise implementation of the sensor component.
   * \param deleter Callback function to be called upon destruction.
   */
  HARDWARE_INTERFACE_PUBLIC
  Sensor(std::shared_ptr<SensorInterface> impl, Deleter deleter);

  HARDWARE_INTERFACE_PUBLIC
  ~Sensor();

  /// Access the underlying implementation
  /**
   * The function returns a reference to the implementation.
   * This allows for a full-access API for optimal use.
   *
   * \note: The given template parameter has to be directly convertible.
   * A wrongly specified template argument might lead to UB.
   * \return Reference to the casted interface.
   */
  template<class ImplT>
  auto & as() {return *std::static_pointer_cast<ImplT>(impl_);}

  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const ComponentInfo & sensor_info);

  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string> get_state_interfaces() const;

  HARDWARE_INTERFACE_PUBLIC
  return_type get_state(
    std::vector<double> & state, const std::vector<std::string> & interfaces) const;

  HARDWARE_INTERFACE_PUBLIC
  return_type get_state(std::vector<double> & state) const;

  HARDWARE_INTERFACE_PUBLIC
  return_type set_state(
    const std::vector<double> & state, const std::vector<std::string> & interfaces);

  HARDWARE_INTERFACE_PUBLIC
  return_type set_state(const std::vector<double> & state);

private:
  std::shared_ptr<SensorInterface> impl_;
  Deleter deleter_;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_
