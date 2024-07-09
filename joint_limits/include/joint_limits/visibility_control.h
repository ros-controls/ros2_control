// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef JOINT_LIMITS__VISIBILITY_CONTROL_H_
#define JOINT_LIMITS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JOINT_LIMITS_EXPORT __attribute__((dllexport))
#define JOINT_LIMITS_IMPORT __attribute__((dllimport))
#else
#define JOINT_LIMITS_EXPORT __declspec(dllexport)
#define JOINT_LIMITS_IMPORT __declspec(dllimport)
#endif
#ifdef JOINT_LIMITS_BUILDING_DLL
#define JOINT_LIMITS_PUBLIC JOINT_LIMITS_EXPORT
#else
#define JOINT_LIMITS_PUBLIC JOINT_LIMITS_IMPORT
#endif
#define JOINT_LIMITS_PUBLIC_TYPE JOINT_LIMITS_PUBLIC
#define JOINT_LIMITS_LOCAL
#else
#define JOINT_LIMITS_EXPORT __attribute__((visibility("default")))
#define JOINT_LIMITS_IMPORT
#if __GNUC__ >= 4
#define JOINT_LIMITS_PUBLIC __attribute__((visibility("default")))
#define JOINT_LIMITS_LOCAL __attribute__((visibility("hidden")))
#else
#define JOINT_LIMITS_PUBLIC
#define JOINT_LIMITS_LOCAL
#endif
#define JOINT_LIMITS_PUBLIC_TYPE
#endif

#endif  // JOINT_LIMITS__VISIBILITY_CONTROL_H_
