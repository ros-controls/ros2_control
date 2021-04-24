// Copyright 2020 PAL Robotics S.L.
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

#ifndef JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_EXCEPTION_HPP_
#define JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_EXCEPTION_HPP_

#include <string>

namespace joint_limits_interface
{
/// An exception related to a \ref JointLimitsInterface
class JointLimitsInterfaceException : public std::exception
{
public:
  explicit JointLimitsInterfaceException(const std::string & message) : msg(message) {}

  const char * what() const noexcept override { return msg.c_str(); }

private:
  std::string msg;
};

}  // namespace joint_limits_interface

#endif  // JOINT_LIMITS_INTERFACE__JOINT_LIMITS_INTERFACE_EXCEPTION_HPP_
