// Copyright 2020 ROS2-Control Development Team
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


#ifndef ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HPP_
#define ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HPP_

#include "pluginlib/class_loader.hpp"
#include "ros2_control_core/visibility_control.h"

namespace ros2_control_core
{

template < typename ROS2ControlType >
class ROS2ControlLoaderPluginlib
{
public:
  ROS2_CONTROL_CORE_PUBLIC ROS2ControlLoaderPluginlib(const std::string package, const std::string base_type) : loader_(std::make_shared<pluginlib::ClassLoader< ROS2ControlType >>(package, base_type))
  {
  };

  ROS2_CONTROL_CORE_PUBLIC
  virtual ~ROS2ControlLoaderPluginlib() = default;

  // TODO: Add try-catch
  ROS2_CONTROL_CORE_PUBLIC
  std::shared_ptr<ROS2ControlType> create(const std::string & type)
  {
    return std::shared_ptr<ROS2ControlType>(loader_->createUnmanagedInstance(type));
  };

  ROS2_CONTROL_CORE_PUBLIC
  bool is_available(const std::string & type)
  {
    return loader_->isClassAvailable(type);
  };

private:
  std::shared_ptr<pluginlib::ClassLoader< ROS2ControlType >> loader_;
};

}  // namespace ros2_control_core

#endif  // ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HPP_
