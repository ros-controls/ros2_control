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

#include "hardware_interface/introspection.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
struct ControllerInterfaceBase::ControllerInterfaceBaseImpl
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<return_type>> async_handler_;
  bool is_async_ = false;
  controller_interface::ControllerInterfaceParams ctrl_itf_params_;
  std::atomic_bool skip_async_triggers_ = false;
  ControllerUpdateStats trigger_stats_;
  mutable std::atomic<uint8_t> lifecycle_id_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
};

ControllerInterfaceBase::ControllerInterfaceBase()
: impl_(std::make_unique<ControllerInterfaceBaseImpl>())
{
}

ControllerInterfaceBase::~ControllerInterfaceBase()
{
  // check if node is initialized and we still have a valid context
  if (
    impl_->node_.get() &&
    get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED &&
    rclcpp::ok())
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Calling shutdown transition of controller node '%s' due to destruction.",
      get_node()->get_name());
    impl_->node_->shutdown();
  }
}

return_type ControllerInterfaceBase::init(
  const std::string & controller_name, const std::string & urdf, unsigned int cm_update_rate,
  const std::string & node_namespace, const rclcpp::NodeOptions & node_options)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = controller_name;
  params.robot_description = urdf;
  params.update_rate = cm_update_rate;
  params.controller_manager_update_rate = cm_update_rate;
  params.node_namespace = node_namespace;
  params.node_options = node_options;

  return init(params);
}

return_type ControllerInterfaceBase::init(
  const controller_interface::ControllerInterfaceParams & params)
{
  impl_->ctrl_itf_params_ = params;
  impl_->node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    params.controller_name, params.node_namespace, params.node_options,
    false);  // disable LifecycleNode service interfaces
  impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);

  if (params.controller_manager_update_rate == 0 && params.update_rate != 0)
  {
    RCLCPP_WARN(
      impl_->node_->get_logger(), "%s",
      fmt::format(
        "The 'controller_manager_update_rate' variable of the ControllerInterfaceParams is unset "
        "or set to 0 Hz while the 'update_rate' variable is set to a non-zero value of '{} Hz'. "
        "Using the controller's update rate as the controller manager update rate. Please fix in "
        "the tests by initializing the 'controller_manager_update_rate' instead of the "
        "'update_rate' variable within the ControllerInterfaceParams struct",
        params.update_rate)
        .c_str());
    impl_->ctrl_itf_params_.controller_manager_update_rate = params.update_rate;
  }

  try
  {
    // no rclcpp::ParameterValue unsigned int specialization
    auto_declare<int>("update_rate", static_cast<int>(params.controller_manager_update_rate));
    auto_declare<bool>("is_async", false);
    auto_declare<int>("thread_priority", -100);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return return_type::ERROR;
  }

  impl_->node_->register_on_configure(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      return on_configure(previous_state);
    });

  impl_->node_->register_on_cleanup(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      // make sure introspection is disabled on controller cleanup as users may manually enable
      // it in `on_configure` and `on_deactivate` - see the docs for details
      enable_introspection(false);
      this->stop_async_handler_thread();
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      return on_cleanup(previous_state);
    });

  impl_->node_->register_on_activate(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      impl_->skip_async_triggers_.store(false, std::memory_order_release);
      enable_introspection(true);
      if (is_async() && impl_->async_handler_ && impl_->async_handler_->is_running())
      {
        // This is needed if it is disabled due to a thrown exception in the async callback thread
        impl_->async_handler_->reset_variables();
      }
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      return on_activate(previous_state);
    });

  impl_->node_->register_on_deactivate(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      enable_introspection(false);
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      return on_deactivate(previous_state);
    });

  impl_->node_->register_on_shutdown(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      this->stop_async_handler_thread();
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      auto transition_state_status = on_shutdown(previous_state);
      this->release_interfaces();
      return transition_state_status;
    });

  impl_->node_->register_on_error(
    [this](const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
    {
      this->stop_async_handler_thread();
      impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
      auto transition_state_status = on_error(previous_state);
      this->release_interfaces();
      return transition_state_status;
    });

  switch (on_init())
  {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Calling shutdown transition of controller node '%s' due to init failure.",
        get_node()->get_name());
      impl_->node_->shutdown();
      return return_type::ERROR;
  }

  return return_type::OK;
}

const rclcpp_lifecycle::State & ControllerInterfaceBase::configure()
{
  impl_->lifecycle_id_.store(this->get_lifecycle_state().id(), std::memory_order_release);
  auto & params = impl_->ctrl_itf_params_;
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
    if (params.update_rate != 0u && update_rate > params.update_rate)
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "%s",
        fmt::format(
          "The update rate of the controller : '{} Hz' cannot be higher than the update rate of "
          "the controller manager : '{} Hz'. Setting it to the update rate of the controller "
          "manager.",
          update_rate, params.update_rate)
          .c_str());
    }
    else
    {
      if (update_rate > 0 && params.controller_manager_update_rate > 0)
      {
        // Calculate the update rate corresponding the periodicity of the controller manager
        const bool is_frequency_achievable =
          (params.controller_manager_update_rate % static_cast<unsigned int>(update_rate)) == 0;
        const unsigned int ticks_per_controller_per_second = static_cast<unsigned int>(std::round(
          static_cast<double>(params.controller_manager_update_rate) /
          static_cast<double>(update_rate)));
        const unsigned int achievable_hz =
          is_frequency_achievable
            ? static_cast<unsigned int>(update_rate)
            : params.controller_manager_update_rate / ticks_per_controller_per_second;

        RCLCPP_WARN_EXPRESSION(
          get_node()->get_logger(), !is_frequency_achievable, "%s",
          fmt::format(
            "The requested update rate of '{}' Hz is not achievable with the controller manager "
            "update rate of '{}' Hz. Setting it to the closest achievable frequency '{}' Hz.",
            update_rate, params.controller_manager_update_rate, achievable_hz)
            .c_str());
        params.update_rate = achievable_hz;
      }
    }
    impl_->is_async_ = get_node()->get_parameter("is_async").as_bool();
  }
  if (impl_->is_async_)
  {
    realtime_tools::AsyncFunctionHandlerParams async_params;
    async_params.thread_priority = 50;  // default value
    const int thread_priority_param =
      static_cast<int>(get_node()->get_parameter("thread_priority").as_int());
    if (thread_priority_param >= 0 && thread_priority_param <= 99)
    {
      async_params.thread_priority = thread_priority_param;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "The parsed 'thread_priority' parameter will be deprecated and not be functional from "
        "ROS 2 Lyrical Luth release. Please use the 'async_parameters.thread_priority' parameter "
        "instead.");
    }
    async_params.initialize(impl_->node_, "async_parameters.");
    if (async_params.scheduling_policy == realtime_tools::AsyncSchedulingPolicy::DETACHED)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The controllers are not supported to run asynchronously in detached mode!");
      return get_node()->get_current_state();
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "Starting async handler with scheduler priority: %d",
      async_params.thread_priority);
    impl_->async_handler_ = std::make_unique<realtime_tools::AsyncFunctionHandler<return_type>>();
    impl_->async_handler_->init(
      std::bind(
        &ControllerInterfaceBase::update, this, std::placeholders::_1, std::placeholders::_2),
      async_params);
    impl_->async_handler_->start_thread();
  }
  REGISTER_ROS2_CONTROL_INTROSPECTION("total_triggers", &impl_->trigger_stats_.total_triggers);
  REGISTER_ROS2_CONTROL_INTROSPECTION("failed_triggers", &impl_->trigger_stats_.failed_triggers);
  impl_->trigger_stats_.reset();

  const auto & return_value = get_node()->configure();
  impl_->lifecycle_id_.store(return_value.id(), std::memory_order_release);
  return return_value;
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
  if (!impl_->node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return impl_->node_->get_current_state();
}

uint8_t ControllerInterfaceBase::get_lifecycle_id() const
{
  const auto id = impl_->lifecycle_id_.load(std::memory_order_acquire);
  if (
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING ||
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING ||
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP ||
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING ||
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN ||
    id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    const auto new_id = this->get_lifecycle_state().id();
    impl_->lifecycle_id_.store(new_id, std::memory_order_release);
    return new_id;
  }
  return id;
}

ControllerUpdateStatus ControllerInterfaceBase::trigger_update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ControllerUpdateStatus status;
  impl_->trigger_stats_.total_triggers++;
  if (is_async())
  {
    if (impl_->skip_async_triggers_.load())
    {
      // Skip further async triggers if the controller is being deactivated
      status.successful = false;
      status.result = return_type::OK;
      return status;
    }
    const rclcpp::Time last_trigger_time = impl_->async_handler_->get_current_callback_time();
    const auto result = impl_->async_handler_->trigger_async_callback(time, period);
    if (!result.first)
    {
      impl_->trigger_stats_.failed_triggers++;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 20000,
        "The controller missed %u update cycles out of %u total triggers.",
        impl_->trigger_stats_.failed_triggers, impl_->trigger_stats_.total_triggers);
    }
    status.successful = result.first;
    status.result = result.second;
    const auto execution_time = impl_->async_handler_->get_last_execution_time();
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
  if (!impl_->node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return impl_->node_;
}

std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node() const
{
  if (!impl_->node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return impl_->node_;
}

unsigned int ControllerInterfaceBase::get_update_rate() const
{
  return impl_->ctrl_itf_params_.update_rate;
}

bool ControllerInterfaceBase::is_async() const { return impl_->is_async_; }

const std::string & ControllerInterfaceBase::get_robot_description() const
{
  return impl_->ctrl_itf_params_.robot_description;
}

const std::unordered_map<std::string, joint_limits::JointLimits> &
ControllerInterfaceBase::get_hard_joint_limits() const
{
  return impl_->ctrl_itf_params_.hard_joint_limits;
}

const std::unordered_map<std::string, joint_limits::SoftJointLimits> &
ControllerInterfaceBase::get_soft_joint_limits() const
{
  return impl_->ctrl_itf_params_.soft_joint_limits;
}

void ControllerInterfaceBase::wait_for_trigger_update_to_finish()
{
  if (is_async() && impl_->async_handler_ && impl_->async_handler_->is_running())
  {
    impl_->async_handler_->wait_for_trigger_cycle_to_finish();
  }
}

void ControllerInterfaceBase::prepare_for_deactivation()
{
  impl_->skip_async_triggers_.store(true, std::memory_order_release);
  this->wait_for_trigger_update_to_finish();
}

void ControllerInterfaceBase::stop_async_handler_thread()
{
  if (is_async() && impl_->async_handler_ && impl_->async_handler_->is_running())
  {
    impl_->async_handler_->stop_thread();
  }
}

std::string ControllerInterfaceBase::get_name() const { return get_node()->get_name(); }

void ControllerInterfaceBase::enable_introspection(bool enable)
{
  if (enable)
  {
    stats_registrations_.enableAll();
  }
  else
  {
    stats_registrations_.disableAll();
  }
}

}  // namespace controller_interface
