// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RESOURCE_MANAGER_HPP_
#define RESOURCE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/components/sensor.hpp"

namespace controller_manager
{

class ResourceStorage;

class ResourceManager
{
public:
  ResourceManager();

  explicit ResourceManager(const std::string & urdf);

  ResourceManager(const ResourceManager & other) = delete;

  ~ResourceManager();

  /// Return the number of joint components
  /**
   * \return number of joints
   */
  size_t joint_components_size() const;

  /// Return the names of joint components
  /**
   * \return vector of joint names
   */
  std::vector<std::string> joint_components_name() const;

  /// Return the number of sensor components
  /**
   * \return number of sensors
   */
  size_t sensor_components_size() const;

  /// Return the names of sensor components
  /**
   * \return vector of sensor names
   */
  std::vector<std::string> sensor_components_name() const;

  /// Return the number of hardware actuators
  /**
   * \return number of actuators
   */
  size_t actuator_interfaces_size() const;

  /// Return the names of hardware actuators
  /**
   * \return vector of actuator names
   */
  std::vector<std::string> actuator_interfaces_name() const;

  /// Return the number of hardware sensors
  /**
   * \return number of sensors
   */
  size_t sensor_interfaces_size() const;

  /// Return the names of hardware sensors
  /**
   * \return vector of sensor names
   */
  std::vector<std::string> sensor_interfaces_name() const;

  /// Return the number of hardware systems
  /**
   * \return number of sysytems
   */
  size_t system_interfaces_size() const;

  /// Return the names of hardware systems
  /**
   * \return vector of system names
   */
  std::vector<std::string> system_interfaces_name() const;

  /// Claim a sensor component based on its id
  /**
   * The resource is claimed as long as being in scope.
   * Once the resource is going out of scope, the destructor
   * returns and thus frees the resource to claimed by others.
   *
   * \param sensor_id String identifier which sensor to claim
   * \return Sensor claimed sensor component
   */
  hardware_interface::components::Sensor claim_sensor(const std::string & sensor_id);

  /// Verify if a sensor exist with a given id
  /**
   * Before claiming a sensor, it might be worth to check if the sensor exists.
   * \param sensor_id String identifier which sensor to look for
   * \return bool True if sensor exists, else false
   */
  bool sensor_exists(const std::string & sensor_id) const;

  /// Verify if a sensor is claimed
  /**
   * Sensors can only uniquely be claimed.
   *
   * \param sensor_id String identifier which sensor to look for
   * \return bool True if component is claimed, else false
   */
  bool sensor_is_claimed(const std::string & sensor_id) const;

private:
  void release_component(const std::string & component_id);

  // TODO(karsten1987): Optimize this with std::vector, maps are bad
  std::unordered_map<std::string, bool> claimed_resource_map_;

  mutable std::recursive_mutex resource_lock_;
  std::unique_ptr<ResourceStorage> resource_storage_;
};

}  // namespace controller_manager
#endif  // RESOURCE_MANAGER_HPP_
