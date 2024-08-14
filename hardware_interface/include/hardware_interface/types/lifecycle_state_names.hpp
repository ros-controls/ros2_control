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
//
/// \author: Denis Stogl

#ifndef HARDWARE_INTERFACE__TYPES__LIFECYCLE_STATE_NAMES_HPP_
#define HARDWARE_INTERFACE__TYPES__LIFECYCLE_STATE_NAMES_HPP_

namespace hardware_interface
{
namespace lifecycle_state_names
{
/// Constants defining string labels corresponding to lifecycle states
constexpr char UNKNOWN[] = "unknown";
constexpr char UNCONFIGURED[] = "unconfigured";
constexpr char INACTIVE[] = "inactive";
constexpr char ACTIVE[] = "active";
constexpr char FINALIZED[] = "finalized";
}  // namespace lifecycle_state_names

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__LIFECYCLE_STATE_NAMES_HPP_
