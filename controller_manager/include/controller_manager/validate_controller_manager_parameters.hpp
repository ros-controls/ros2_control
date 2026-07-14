// Copyright 2026 ros2_control Development Team
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

#ifndef CONTROLLER_MANAGER__VALIDATE_CONTROLLER_MANAGER_PARAMETERS_HPP_
#define CONTROLLER_MANAGER__VALIDATE_CONTROLLER_MANAGER_PARAMETERS_HPP_

#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/parameter.hpp"
#include "rsl/parameter_validators.hpp"

namespace controller_manager
{
inline tl::expected<void, std::string> fits_update_rate_storage(const rclcpp::Parameter & parameter)
{
  const auto update_rate = parameter.as_int();
  const auto max_update_rate = static_cast<std::uint64_t>(std::numeric_limits<unsigned int>::max());

  if (update_rate > 0 && static_cast<std::uint64_t>(update_rate) > max_update_rate)
  {
    return tl::make_unexpected("must be less than or equal to " + std::to_string(max_update_rate));
  }

  return {};
}
}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__VALIDATE_CONTROLLER_MANAGER_PARAMETERS_HPP_
