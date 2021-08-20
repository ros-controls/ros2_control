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

#ifndef FAKE_COMPONENTS__VISIBILITY_CONTROL_H_
#define FAKE_COMPONENTS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define FAKE_COMPONENTS_EXPORT __attribute__((dllexport))
#define FAKE_COMPONENTS_IMPORT __attribute__((dllimport))
#else
#define FAKE_COMPONENTS_EXPORT __declspec(dllexport)
#define FAKE_COMPONENTS_IMPORT __declspec(dllimport)
#endif
#ifdef FAKE_COMPONENTS_BUILDING_DLL
#define FAKE_COMPONENTS_PUBLIC FAKE_COMPONENTS_EXPORT
#else
#define FAKE_COMPONENTS_PUBLIC FAKE_COMPONENTS_IMPORT
#endif
#define FAKE_COMPONENTS_PUBLIC_TYPE FAKE_COMPONENTS_PUBLIC
#define FAKE_COMPONENTS_LOCAL
#else
#define FAKE_COMPONENTS_EXPORT __attribute__((visibility("default")))
#define FAKE_COMPONENTS_IMPORT
#if __GNUC__ >= 4
#define FAKE_COMPONENTS_PUBLIC __attribute__((visibility("default")))
#define FAKE_COMPONENTS_LOCAL __attribute__((visibility("hidden")))
#else
#define FAKE_COMPONENTS_PUBLIC
#define FAKE_COMPONENTS_LOCAL
#endif
#define FAKE_COMPONENTS_PUBLIC_TYPE
#endif

#endif  // FAKE_COMPONENTS__VISIBILITY_CONTROL_H_
