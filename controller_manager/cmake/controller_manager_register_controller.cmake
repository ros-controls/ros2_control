# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake_core REQUIRED)

macro(controller_manager_register_controller target)
  if(NOT TARGET ${target})
    message(
      FATAL_ERROR
      "controller_manager_register_controller() first argument "
      "'${target}' is not a target")
  endif()

  cmake_parse_arguments(_ARG "SKIP_INSTALL" "" "" ${ARGN})

  get_target_property(_target_type ${target} TYPE)
  if(NOT _target_type STREQUAL "SHARED_LIBRARY")
    message(
      FATAL_ERROR
      "controller_manager_register_controller() first argument "
      "'${target}' is not a shared library target")
  endif()
  set(_unique_names)
  foreach(_arg ${_ARG_UNPARSED_ARGUMENTS})
    if(_arg IN_LIST _unique_names)
      message(
        FATAL_ERROR
        "controller_manager_register_controller() the plugin names "
        "must be unique (multiple '${_arg}')")
    endif()
    list(APPEND _unique_names "${_arg}")

    if(_ARG_SKIP_INSTALL)
      set(_library_path $<TARGET_FILE:${target}>)
    else()
      if(WIN32)
        set(_path "bin")
      else()
        set(_path "lib")
      endif()
      set(_library_path ${_path}/$<TARGET_FILE_NAME:${target}>)
    endif()
    set(_ROS_CONTROLLERS
      "${_ROS_CONTROLLERS}${_arg};${_library_path}\n")
  endforeach()

  if(_ARG_SKIP_INSTALL)
    ament_index_register_resource("ros_controllers" CONTENT ${_ROS_CONTROLLERS} AMENT_INDEX_BINARY_DIR "${CMAKE_BINARY_DIR}/ament_cmake_index_$<CONFIG>" SKIP_INSTALL)
  else()
    ament_index_register_resource("ros_controllers" CONTENT ${_ROS_CONTROLLERS})
  endif()
endmacro()
