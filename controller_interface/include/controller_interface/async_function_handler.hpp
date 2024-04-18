// Copyright 2024 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#ifndef CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_
#define CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ros2_control
{
/**
 * @brief Class to handle asynchronous function calls.
 * AsyncFunctionHandler is a class that allows the user to have a ascynchronous call to the parsed
 * method, when the lifecycyle state is in the ACTIVE state
 */
template <typename T>
class AsyncFunctionHandler
{
public:
  AsyncFunctionHandler() = default;

  ~AsyncFunctionHandler() { stop_async_update(); }

  /// Initialize the AsyncFunctionHandler with the get_state_function and async_function
  /**
   * @param get_state_function Function that returns the current lifecycle state
   * @param async_function Function that will be called asynchronously
   * If the AsyncFunctionHandler is already initialized and is running, it will throw a runtime
   * error.
   * If the parsed functions are not valid, it will throw a runtime error.
   */
  void init(
    std::function<const rclcpp_lifecycle::State &()> get_state_function,
    std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> async_function)
  {
    if (get_state_function == nullptr)
    {
      throw std::runtime_error(
        "AsyncFunctionHandler: parsed function to get the lifecycle state is not valid!");
    }
    if (async_function == nullptr)
    {
      throw std::runtime_error(
        "AsyncFunctionHandler: parsed function to call asynchronously is not valid!");
    }
    if (thread_.joinable())
    {
      throw std::runtime_error(
        "AsyncFunctionHandler: Cannot reinitialize while the thread is "
        "running. Please stop the async update first!");
    }
    get_state_function_ = get_state_function;
    async_function_ = async_function;
  }

  /// Triggers the async update method cycle
  /**
   * @param time Current time
   * @param period Current period
   * @return A pair with the first element being a boolean indicating if the async update method was
   * triggered and the second element being the last return value of the async function.
   * If the AsyncFunctionHandler is not initialized properly, it will throw a runtime error.
   * If the async update method is waiting for the trigger, it will notify the async thread to start
   * the update cycle.
   * If the async update method is still running, it will return the last return value of the async
   * function.
   *
   * \note In the case of controllers, The controller manager is responsible
   * for triggering and maintaining the controller's update rate, as it should be only acting as a
   * scheduler. Same applies to the resource manager when handling the hardware components.
   */
  std::pair<bool, T> trigger_async_update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    initialize_async_update_thread();
    std::unique_lock<std::mutex> lock(async_mtx_, std::try_to_lock);
    bool trigger_status = false;
    if (!trigger_in_progress_ && lock.owns_lock())
    {
      trigger_in_progress_ = true;
      current_update_time_ = time;
      current_update_period_ = period;
      lock.unlock();
      async_update_condition_.notify_one();
      trigger_status = true;
    }
    const T return_value = async_update_return_;
    return std::make_pair(trigger_status, return_value);
  }

  /// Waits until the current async update method cycle to finish
  /**
   * If the async method is running, it will wait for the current async method call to finish.
   */
  void wait_for_trigger_cycle_to_finish()
  {
    if (is_running())
    {
      std::unique_lock<std::mutex> lock(async_mtx_);
      async_update_condition_.wait(lock, [this] { return !trigger_in_progress_; });
    }
  }

  /// Check if the AsyncFunctionHandler is initialized
  /**
   * @return True if the AsyncFunctionHandler is initialized, false otherwise
   */
  bool is_initialized() const { return get_state_function_ && async_function_; }

  /// Join the async update thread
  /**
   * If the async method is running, it will join the async thread.
   * If the async method is not running, it will return immediately.
   */
  void join_async_update_thread()
  {
    if (is_running())
    {
      thread_.join();
    }
  }

  /// Check if the async update thread is running
  /**
   * @return True if the async update thread is running, false otherwise
   */
  bool is_running() const { return thread_.joinable(); }

  /// Check if the async update is triggered to stop the cycle
  /**
   * @return True if the async update is stopped, false otherwise
   */
  bool is_stopped() const
  {
    return (
      async_update_stop_ ||
      (is_initialized() &&
       get_state_function_().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  }

  /// Stops the async update thread
  /**
   * If the async method is running, it will notify the async thread to stop and then joins the
   * async thread.
   */
  void stop_async_update()
  {
    if (is_running())
    {
      async_update_stop_ = true;
      async_update_condition_.notify_one();
      thread_.join();
    }
  }

private:
  /// Initialize the async update thread
  /**
   * If the async update thread is not running, it will start the async update thread.
   * If the async update thread is already configured and running, does nothing and return
   * immediately.
   */
  void initialize_async_update_thread()
  {
    if (!is_initialized())
    {
      throw std::runtime_error("AsyncFunctionHandler: need to be initialized first!");
    }
    if (!thread_.joinable())
    {
      async_update_stop_ = false;
      trigger_in_progress_ = false;
      async_update_return_ = T();
      thread_ = std::thread(
        [this]() -> void
        {
          // \note There might be an concurrency issue with the get_state() call here. This mightn't
          // be critical here as the state of the controller is not expected to change during the
          // update cycle
          while (get_state_function_().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
                 !async_update_stop_)
          {
            std::unique_lock<std::mutex> lock(async_mtx_);
            async_update_condition_.wait(
              lock, [this] { return trigger_in_progress_ || async_update_stop_; });
            if (async_update_stop_)
            {
              break;
            }
            async_update_return_ = async_function_(current_update_time_, current_update_period_);
            trigger_in_progress_ = false;
            lock.unlock();
            async_update_condition_.notify_one();
          }
        });
    }
  }

  rclcpp::Time current_update_time_;
  rclcpp::Duration current_update_period_{0, 0};

  std::function<const rclcpp_lifecycle::State &()> get_state_function_;
  std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> async_function_;

  // Async related variables
  std::thread thread_;
  std::atomic_bool async_update_stop_{false};
  std::atomic_bool trigger_in_progress_{false};
  std::atomic<T> async_update_return_;
  std::condition_variable async_update_condition_;
  std::mutex async_mtx_;
};
}  // namespace ros2_control

#endif  // CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_
