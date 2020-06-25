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

#ifndef HARDWARE_INTERFACE__OPERATION_MODE_HANDLE_HPP_
#define HARDWARE_INTERFACE__OPERATION_MODE_HANDLE_HPP_

#include <string>

#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// Enum for Operation Mode.
/** Operation can either be active or inactive */
enum class HARDWARE_INTERFACE_PUBLIC_TYPE OperationMode: bool
{
  ACTIVE = true,
  INACTIVE = false
};

/// A handle for Operation Mode.
/** Used to set status to active or inactive for operation such as read/write. */
class OperationModeHandle
{
public:
  HARDWARE_INTERFACE_PUBLIC
  OperationModeHandle();

  /// Constructor of Operation Mode.
  /**
   * The mode handles are passive in terms of ownership for the mode pointer.
   * This means the handle is allowed to read and write the respective mode,
   * however is not allowed to reallocate or delete the memory storage.
   * \param name The name of the joint
   * \param mode A pointer to the storage for this hardware's operation mode
   */
  HARDWARE_INTERFACE_PUBLIC
  OperationModeHandle(
    const std::string & name,
    OperationMode * mode);

  HARDWARE_INTERFACE_PUBLIC
  const std::string &
  get_name() const;

  HARDWARE_INTERFACE_PUBLIC
  void
  set_mode(OperationMode mode);

  HARDWARE_INTERFACE_PUBLIC
  bool valid_pointers() const;

private:
  std::string name_;
  OperationMode * mode_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__OPERATION_MODE_HANDLE_HPP_
