// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "controller_interface/controller_interface_base.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
ControllerInterfaceBase::~ControllerInterfaceBase()
{
  // check if node is initialized and we still have a valid context
  if (
    node_.get() &&
    get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED &&
    rclcpp::ok())
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Calling shutdown transition of controller node '%s' due to destruction.",
      get_node()->get_name());
    node_->shutdown();
  }
}

return_type ControllerInterfaceBase::init(
  const std::string & controller_name, const std::string & urdf, unsigned int cm_update_rate,
  const std::string & node_namespace, const rclcpp::NodeOptions & node_options)
{
  urdf_ = urdf;
  update_rate_ = cm_update_rate;
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    controller_name, node_namespace, node_options,
    false);  // disable LifecycleNode service interfaces

  try
  {
    // no rclcpp::ParameterValue unsigned int specialization
    auto_declare<int>("update_rate", static_cast<int>(update_rate_));

    auto_declare<bool>("is_async", false);
    auto_declare<int>("thread_priority", 50);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return return_type::ERROR;
  }

  switch (on_init())
  {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Calling shutdown transition of controller node '%s' due to init failure.",
        get_node()->get_name());
      node_->shutdown();
      return return_type::ERROR;
  }

  node_->register_on_configure(
    std::bind(&ControllerInterfaceBase::on_configure, this, std::placeholders::_1));

  node_->register_on_cleanup(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      if (is_async() && async_handler_ && async_handler_->is_running())
      {
        async_handler_->stop_thread();
      }
      return on_cleanup(previous_state);
    });

  node_->register_on_activate(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      if (is_async() && async_handler_ && async_handler_->is_running())
      {
        // This is needed if it is disabled due to a thrown exception in the async callback thread
        async_handler_->reset_variables();
      }
      return on_activate(previous_state);
    });

  node_->register_on_deactivate(
    std::bind(&ControllerInterfaceBase::on_deactivate, this, std::placeholders::_1));

  node_->register_on_shutdown(
    std::bind(&ControllerInterfaceBase::on_shutdown, this, std::placeholders::_1));

  node_->register_on_error(
    std::bind(&ControllerInterfaceBase::on_error, this, std::placeholders::_1));

  return return_type::OK;
}

const rclcpp_lifecycle::State & ControllerInterfaceBase::configure()
{
  // TODO(destogl): this should actually happen in "on_configure" but I am not sure how to get
  // overrides correctly in combination with std::bind. The goal is to have the following calls:
  // 1. CM: controller.get_node()->configure()
  // 2. LifecycleNode: ControllerInterfaceBase::on_configure()
  // 3. ControllerInterfaceBase: <controller>::on_configure()
  // Then we don't need to do state-machine related checks.
  //
  // Other solution is to add check into the LifecycleNode if a transition is valid to trigger
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    const auto update_rate = get_node()->get_parameter("update_rate").as_int();
    if (update_rate < 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Update rate cannot be a negative value!");
      return get_lifecycle_state();
    }
    if (update_rate_ != 0u && update_rate > update_rate_)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "The update rate of the controller : '%ld Hz' cannot be higher than the update rate of the "
        "controller manager : '%d Hz'. Setting it to the update rate of the controller manager.",
        update_rate, update_rate_);
    }
    else
    {
      update_rate_ = static_cast<unsigned int>(update_rate);
    }
    is_async_ = get_node()->get_parameter("is_async").as_bool();
  }
  if (is_async_)
  {
    const int thread_priority =
      static_cast<int>(get_node()->get_parameter("thread_priority").as_int());
    RCLCPP_INFO(
      get_node()->get_logger(), "Starting async handler with scheduler priority: %d",
      thread_priority);
    async_handler_ = std::make_unique<realtime_tools::AsyncFunctionHandler<return_type>>();
    async_handler_->init(
      std::bind(
        &ControllerInterfaceBase::update, this, std::placeholders::_1, std::placeholders::_2),
      thread_priority);
    async_handler_->start_thread();
  }
  trigger_stats_.reset();

  return get_node()->configure();
}

void ControllerInterfaceBase::assign_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
  std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
{
  command_interfaces_ = std::forward<decltype(command_interfaces)>(command_interfaces);
  state_interfaces_ = std::forward<decltype(state_interfaces)>(state_interfaces);
}

void ControllerInterfaceBase::release_interfaces()
{
  command_interfaces_.clear();
  state_interfaces_.clear();
}

const rclcpp_lifecycle::State & ControllerInterfaceBase::get_lifecycle_state() const
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_->get_current_state();
}

ControllerUpdateStatus ControllerInterfaceBase::trigger_update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ControllerUpdateStatus status;
  trigger_stats_.total_triggers++;
  if (is_async())
  {
    const rclcpp::Time last_trigger_time = async_handler_->get_current_callback_time();
    const auto result = async_handler_->trigger_async_callback(time, period);
    if (!result.first)
    {
      trigger_stats_.failed_triggers++;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 20000,
        "The controller missed %u update cycles out of %u total triggers.",
        trigger_stats_.failed_triggers, trigger_stats_.total_triggers);
    }
    status.successful = result.first;
    status.result = result.second;
    const auto execution_time = async_handler_->get_last_execution_time();
    if (execution_time.count() > 0)
    {
      status.execution_time = execution_time;
    }
    if (last_trigger_time.get_clock_type() != RCL_CLOCK_UNINITIALIZED)
    {
      status.period = time - last_trigger_time;
    }
  }
  else
  {
    const auto start_time = std::chrono::steady_clock::now();
    status.successful = true;
    status.result = update(time, period);
    status.execution_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start_time);
    status.period = period;
  }
  return status;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node()
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node() const
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

unsigned int ControllerInterfaceBase::get_update_rate() const { return update_rate_; }

bool ControllerInterfaceBase::is_async() const { return is_async_; }

const std::string & ControllerInterfaceBase::get_robot_description() const { return urdf_; }

void ControllerInterfaceBase::wait_for_trigger_update_to_finish()
{
  if (is_async() && async_handler_ && async_handler_->is_running())
  {
    async_handler_->wait_for_trigger_cycle_to_finish();
  }
}
}  // namespace controller_interface
