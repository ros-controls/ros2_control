// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__MACROS_HPP_
#define HARDWARE_INTERFACE__MACROS_HPP_

#include <stdexcept>
#include <string>

#include "rcpputils/pointer_traits.hpp"

#ifdef _WIN32
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

#define THROW_ON_NULLPTR(pointer)                                                                  \
  static_assert(                                                                                   \
    rcpputils::is_pointer<typename std::remove_reference<decltype(pointer)>::type>::value,         \
    #pointer " has to be a pointer");                                                              \
  if (!pointer)                                                                                    \
  {                                                                                                \
    throw std::runtime_error(std::string(__PRETTY_FUNCTION__) + " failed. " #pointer " is null."); \
  }

#define THROW_ON_NOT_NULLPTR(pointer)                                                      \
  static_assert(                                                                           \
    rcpputils::is_pointer<typename std::remove_reference<decltype(pointer)>::type>::value, \
    #pointer " has to be a pointer");                                                      \
  if (pointer)                                                                             \
  {                                                                                        \
    throw std::runtime_error(                                                              \
      std::string(__PRETTY_FUNCTION__) + " failed. " #pointer " would leak memory");       \
  }

#endif  // HARDWARE_INTERFACE__MACROS_HPP_
