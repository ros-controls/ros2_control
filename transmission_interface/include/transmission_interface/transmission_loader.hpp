// Copyright 2022 PAL Robotics S.L.
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

#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_

#include <memory>

#include "hardware_interface/hardware_info.hpp"

#include "transmission_interface/transmission.hpp"

namespace transmission_interface
{
/**
 * \brief Abstract interface for loading transmission instances from configuration data.
 *
 * It also provides convenience methods for specific transmission loaders to leverage.
 */
class TransmissionLoader
{
public:
  virtual ~TransmissionLoader() = default;

  virtual std::shared_ptr<Transmission> load(
    const hardware_interface::TransmissionInfo & transmission_info) = 0;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_LOADER_HPP_
