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

  size_t actuator_interfaces_size() const;

  size_t sensor_interfaces_size() const;

  size_t system_interfaces_size() const;

  // loan_joint(const std::string & name);
  // loan_sensor(const std::string & name);

private:
  std::unique_ptr<ResourceStorage> resource_storage_;
};

}  // namespace controller_manager
#endif  // RESOURCE_MANAGER_HPP_
