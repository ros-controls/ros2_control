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

#
# Configures a controller library which implements the controller_interface.
#
# This should be called on any library which is built to implement the rmw API.
# The custom settings are all related to library symbol visibility, see:
#
# https://gcc.gnu.org/wiki/Visibility
# http://www.ibm.com/developerworks/aix/library/au-aix-symbol-visibility/
#
# Thought about using:
#
# http://www.cmake.org/cmake/help/v2.8.8/cmake.html#module:GenerateExportHeader
#
# But it doesn't seem to set the compiler flags correctly on clang and
# also doesn't work very well when the headers and library are in
# different projects like this.
#
# Effectively, using this macro results in the
# CONTROLLER_INTERFACE_BUILDING_DLL definition
# being set on Windows, and the -fvisibility* flags being passed to gcc and
# clang.
#
# @public
#
macro(controller_interface_configure_controller_library library_target)
  if(WIN32)
    # Causes the visibility macros to use dllexport rather than dllimport
    # which is appropriate when building the dll but not consuming it.
    target_compile_definitions(${library_target}
      PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")
  endif()
endmacro()
