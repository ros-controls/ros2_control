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

#ifndef MULTI_INTERFACE_JOINT__VISIBILITY_CONTROL_H_
#define MULTI_INTERFACE_JOINT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MULTI_INTERFACE_JOINT_EXPORT __attribute__ ((dllexport))
    #define MULTI_INTERFACE_JOINT_IMPORT __attribute__ ((dllimport))
  #else
    #define MULTI_INTERFACE_JOINT_EXPORT __declspec(dllexport)
    #define MULTI_INTERFACE_JOINT_IMPORT __declspec(dllimport)
  #endif
  #ifdef MULTI_INTERFACE_JOINT_BUILDING_LIBRARY
    #define MULTI_INTERFACE_JOINT_PUBLIC MULTI_INTERFACE_JOINT_EXPORT
  #else
    #define MULTI_INTERFACE_JOINT_PUBLIC MULTI_INTERFACE_JOINT_IMPORT
  #endif
  #define MULTI_INTERFACE_JOINT_PUBLIC_TYPE MULTI_INTERFACE_JOINT_PUBLIC
  #define MULTI_INTERFACE_JOINT_LOCAL
#else
  #define MULTI_INTERFACE_JOINT_EXPORT __attribute__ ((visibility("default")))
  #define MULTI_INTERFACE_JOINT_IMPORT
  #if __GNUC__ >= 4
    #define MULTI_INTERFACE_JOINT_PUBLIC __attribute__ ((visibility("default")))
    #define MULTI_INTERFACE_JOINT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MULTI_INTERFACE_JOINT_PUBLIC
    #define MULTI_INTERFACE_JOINT_LOCAL
  #endif
  #define MULTI_INTERFACE_JOINT_PUBLIC_TYPE
#endif

#endif  // MULTI_INTERFACE_JOINT__VISIBILITY_CONTROL_H_
