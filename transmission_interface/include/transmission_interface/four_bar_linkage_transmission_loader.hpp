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

#ifndef TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_LOADER_HPP_
#define TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_LOADER_HPP_

#include <memory>

#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_loader.hpp"

namespace transmission_interface
{
/**
 * \brief Class for loading a four-bar linkage transmission instance from configuration data.
 */
class FourBarLinkageTransmissionLoader : public TransmissionLoader
{
public:
  std::shared_ptr<Transmission> load(
    const hardware_interface::TransmissionInfo & transmission_info) override;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__FOUR_BAR_LINKAGE_TRANSMISSION_LOADER_HPP_
