// Copyright 2026 AIT - Austrian Institute of Technology GmbH
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

#ifndef CONTROLLER_INTERFACE__TEST_UTILS_HPP_
#define CONTROLLER_INTERFACE__TEST_UTILS_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
using lifecycle_msgs::msg::State;

/**
 * @brief Triggers the controller's configure transition and checks success
 *
 * @param controller The controller to test
 * @return true if the controller successfully transitions to the expected state, false if it fails
 * to
 *
 * @throws std::runtime_error if the controller transitions to an unexpected state
 */
template <typename T>
bool configure_succeeds(const std::unique_ptr<T> & controller)
{
  auto state = controller->configure();

  switch (state.id())
  {
    case State::PRIMARY_STATE_INACTIVE:
      return true;
    case State::PRIMARY_STATE_UNCONFIGURED:
      return false;
    default:
      throw std::runtime_error(
        "Unexpected controller state in configure_succeeds: " + std::to_string(state.id()));
  }
}

/**
 * @brief Triggers the controller's activate transition and checks success
 *
 * @param controller The controller to test
 * @return true if the controller successfully transitions to the expected state, false if it fails
 * to
 *
 * @throws std::runtime_error if the controller transitions to an unexpected state
 */
template <typename T>
bool activate_succeeds(const std::unique_ptr<T> & controller)
{
  auto state = controller->get_node()->activate();

  switch (state.id())
  {
    case State::PRIMARY_STATE_ACTIVE:
      return true;
    case State::PRIMARY_STATE_UNCONFIGURED:
      return false;
    default:
      throw std::runtime_error(
        "Unexpected controller state in activate_succeeds: " + std::to_string(state.id()));
  }
}

/**
 * @brief Triggers the controller's cleanup transition and checks success
 *
 * @param controller The controller to test
 * @return true if the controller successfully transitions to the expected state, false if it fails
 * to
 *
 * @throws std::runtime_error if the controller transitions to an unexpected state
 */
template <typename T>
bool deactivate_succeeds(const std::unique_ptr<T> & controller)
{
  auto state = controller->get_node()->deactivate();

  switch (state.id())
  {
    case State::PRIMARY_STATE_INACTIVE:
      return true;
    case State::PRIMARY_STATE_ACTIVE:
      return false;
    default:
      throw std::runtime_error(
        "Unexpected controller state in deactivate_succeeds: " + std::to_string(state.id()));
  }
}

/**
 * @brief Triggers the controller's cleanup transition and checks success
 *
 * @param controller The controller to test
 * @return true if the controller successfully transitions to the expected state, false if it fails
 * to
 *
 * @throws std::runtime_error if the controller transitions to an unexpected state
 */
template <typename T>
bool cleanup_succeeds(const std::unique_ptr<T> & controller)
{
  auto state = controller->get_node()->cleanup();

  switch (state.id())
  {
    case State::PRIMARY_STATE_UNCONFIGURED:
      return true;
    case State::PRIMARY_STATE_INACTIVE:
      return false;
    default:
      throw std::runtime_error(
        "Unexpected controller state in cleanup_succeeds: " + std::to_string(state.id()));
  }
}

/**
 * @brief Triggers the controller's shutdown transition and checks success
 *
 * @param controller The controller to test
 * @return true if the controller successfully transitions to the expected state, false if it fails
 * to
 *
 * @throws std::runtime_error if the controller transitions to an unexpected state
 */
template <typename T>
bool shutdown_succeeds(const std::unique_ptr<T> & controller)
{
  auto state = controller->get_node()->shutdown();

  switch (state.id())
  {
    case State::PRIMARY_STATE_FINALIZED:
      return true;
    case State::PRIMARY_STATE_UNCONFIGURED:
      return false;
    default:
      throw std::runtime_error(
        "Unexpected controller state in shutdown_succeeds: " + std::to_string(state.id()));
  }
}

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__TEST_UTILS_HPP_
