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
#include <vector>

namespace controller_manager
{

class ResourceStorage;

class ResourceManager
{
public:
  ResourceManager();

  explicit ResourceManager(const std::string & urdf);

  ResourceManager(const ResourceManager &) = delete;

  ~ResourceManager();

  /// Returns all registered state handles keys.
  /**
   * The keys are collected from each loaded hardware component.
   *
   * \return vector of strings, containing all registered keys.
   */
  std::vector<std::string> state_handle_keys() const;

  /// Checks whether a handle is registered under the given key.
  /**
   * \return true if handle exist, false otherwise.
   */
  bool state_handle_exists(const std::string & key) const;

  /// Returns all registered command handles keys.
  /**
   * The keys are collected from each loaded hardware component.
   *
   * \return vector of strings, containing all registered keys.
   */
  std::vector<std::string> command_handle_keys() const;

  /// Checks whether a handle is registered under the given key.
  /**
   * \return true if handle exist, false otherwise.
   */
  bool command_handle_exists(const std::string & key) const;

  /// Return the number of loaded actuator components.
  /**
   * \return number of actuator components.
   */
  size_t actuator_interfaces_size() const;

  /// Return the number of loaded sensor components.
  /**
   * \return number of sensor components.
   */
  size_t sensor_interfaces_size() const;

  /// Return the number of loaded system components.
  /**
   * \return number of system components.
   */
  size_t system_interfaces_size() const;

private:
  std::unique_ptr<ResourceStorage> resource_storage_;
};

}  // namespace controller_manager
#endif  // RESOURCE_MANAGER_HPP_
