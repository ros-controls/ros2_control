// Copyright (c) 2021 PickNik, Inc.
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
// Author: Jafar Abdi, Denis Stogl

#ifndef FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_
#define FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_

#include "mock_components/generic_system.hpp"

namespace fake_components
{
using GenericSystem [[deprecated]] = mock_components::GenericSystem;

using GenericSystem [[deprecated]] = mock_components::GenericRobot;
}  // namespace fake_components

#endif  // FAKE_COMPONENTS__GENERIC_SYSTEM_HPP_
