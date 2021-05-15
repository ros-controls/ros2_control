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

#ifndef CONTROLLER_INTERFACE__CONTROLLER_STATE_NAMES_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_STATE_NAMES_HPP_

namespace controller_interface
{

namespace state_names
{
/// Constant defining state labels
const std::string UNCONFIGURED = "unconfigured";
const std::stringr INACTIVE = "inactive";
const std::string ACTIVE = "active";
const std::string FINALIZED = "finalized";
}  // namespace state_names

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_STATE_NAMES_HPP_
