// Copyright 2025 ros2_control Development Team
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

#include "hardware_interface/hardware_component_interface.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node_options.hpp"

namespace hardware_interface
{

class HardwareComponentInterface::HardwareComponentInterfaceImpl
{
public:
  HardwareComponentInterfaceImpl() : logger_(rclcpp::get_logger("hardware_component_interface")) {}

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  rclcpp::Node::SharedPtr hardware_component_node_ = nullptr;
  // interface names to Handle accessed through getters/setters
  std::unordered_map<std::string, StateInterface::SharedPtr> hardware_states_;
  std::unordered_map<std::string, CommandInterface::SharedPtr> hardware_commands_;
  std::atomic<uint8_t> lifecycle_id_cache_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  std::atomic<return_type> read_return_info_ = return_type::OK;
  std::atomic<std::chrono::nanoseconds> read_execution_time_ = std::chrono::nanoseconds::zero();
  std::atomic<return_type> write_return_info_ = return_type::OK;
  std::atomic<std::chrono::nanoseconds> write_execution_time_ = std::chrono::nanoseconds::zero();
};

HardwareComponentInterface::HardwareComponentInterface()
: lifecycle_state_(
    rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN)),
  impl_(std::make_unique<HardwareComponentInterfaceImpl>())
{
}

HardwareComponentInterface::~HardwareComponentInterface() = default;

CallbackReturn HardwareComponentInterface::init(
  const hardware_interface::HardwareComponentParams & params)
{
  impl_->clock_ = params.clock;
  impl_->logger_ = params.logger;
  info_ = params.hardware_info;
  if (params.hardware_info.is_async)
  {
    realtime_tools::AsyncFunctionHandlerParams async_thread_params;
    async_thread_params.thread_priority = info_.async_params.thread_priority;
    async_thread_params.scheduling_policy =
      realtime_tools::AsyncSchedulingPolicy(info_.async_params.scheduling_policy);
    async_thread_params.cpu_affinity_cores = info_.async_params.cpu_affinity_cores;
    async_thread_params.clock = params.clock;
    async_thread_params.logger = get_logger();
    async_thread_params.exec_rate = params.hardware_info.rw_rate;
    async_thread_params.print_warnings = info_.async_params.print_warnings;
    RCLCPP_INFO(
      get_logger(), "Starting async handler with scheduler priority: %d and policy : %s",
      info_.async_params.thread_priority,
      async_thread_params.scheduling_policy.to_string().c_str());
    async_handler_ = std::make_unique<realtime_tools::AsyncFunctionHandler<return_type>>();
    const bool is_sensor_type = (info_.type == "sensor");
    async_handler_->init(
      [this, is_sensor_type](const rclcpp::Time & time, const rclcpp::Duration & period)
      {
        const auto read_start_time = std::chrono::steady_clock::now();
        const auto ret_read = read(time, period);
        const auto read_end_time = std::chrono::steady_clock::now();
        impl_->read_return_info_.store(ret_read, std::memory_order_release);
        impl_->read_execution_time_.store(
          std::chrono::duration_cast<std::chrono::nanoseconds>(read_end_time - read_start_time),
          std::memory_order_release);
        if (ret_read != return_type::OK)
        {
          return ret_read;
        }
        if (
          !is_sensor_type && impl_->lifecycle_id_cache_.load(std::memory_order_acquire) ==
                               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          const auto write_start_time = std::chrono::steady_clock::now();
          const auto ret_write = write(time, period);
          const auto write_end_time = std::chrono::steady_clock::now();
          impl_->write_return_info_.store(ret_write, std::memory_order_release);
          impl_->write_execution_time_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(write_end_time - write_start_time),
            std::memory_order_release);
          return ret_write;
        }
        return return_type::OK;
      },
      async_thread_params);
    async_handler_->start_thread();
  }

  if (auto locked_executor = params.executor.lock())
  {
    std::string node_name = hardware_interface::to_lower_case(params.hardware_info.name);
    std::replace(node_name.begin(), node_name.end(), '/', '_');
    impl_->hardware_component_node_ = std::make_shared<rclcpp::Node>(
      node_name, params.node_namespace, define_custom_node_options());
    locked_executor->add_node(impl_->hardware_component_node_->get_node_base_interface());
  }
  else
  {
    RCLCPP_WARN(
      params.logger,
      "Executor is not available during hardware component initialization for '%s'. Skipping "
      "node creation!",
      params.hardware_info.name.c_str());
  }

  hardware_interface::HardwareComponentInterfaceParams interface_params;
  interface_params.hardware_info = info_;
  interface_params.executor = params.executor;
  return on_init(interface_params);
}

CallbackReturn HardwareComponentInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  info_ = hardware_info;
  if (info_.type == "actuator")
  {
    parse_state_interface_descriptions(info_.joints, joint_state_interfaces_);
    parse_command_interface_descriptions(info_.joints, joint_command_interfaces_);
  }
  else if (info_.type == "sensor")
  {
    parse_state_interface_descriptions(info_.joints, joint_state_interfaces_);
    parse_state_interface_descriptions(info_.sensors, sensor_state_interfaces_);
  }
  else if (info_.type == "system")
  {
    parse_state_interface_descriptions(info_.joints, joint_state_interfaces_);
    parse_state_interface_descriptions(info_.sensors, sensor_state_interfaces_);
    parse_state_interface_descriptions(info_.gpios, gpio_state_interfaces_);
    parse_command_interface_descriptions(info_.joints, joint_command_interfaces_);
    parse_command_interface_descriptions(info_.gpios, gpio_command_interfaces_);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn HardwareComponentInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // This is done for backward compatibility with the old on_init method.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return on_init(params.hardware_info);
#pragma GCC diagnostic pop
}

rclcpp::NodeOptions HardwareComponentInterface::define_custom_node_options() const
{
  rclcpp::NodeOptions node_options;
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 21
  node_options.enable_logger_service(true);
#endif
  return node_options;
}

std::vector<StateInterface> HardwareComponentInterface::export_state_interfaces()
{
  // return empty vector by default. For backward compatibility we try calling
  // export_state_interfaces() and only when empty vector is returned call
  // on_export_state_interfaces()
  return {};
}

std::vector<hardware_interface::InterfaceDescription>
HardwareComponentInterface::export_unlisted_state_interface_descriptions()
{
  // return empty vector by default.
  return {};
}

std::vector<StateInterface::ConstSharedPtr> HardwareComponentInterface::on_export_state_interfaces()
{
  // import the unlisted interfaces
  std::vector<hardware_interface::InterfaceDescription> unlisted_interface_descriptions =
    export_unlisted_state_interface_descriptions();

  std::vector<StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(
    unlisted_interface_descriptions.size() + joint_state_interfaces_.size() +
    sensor_state_interfaces_.size() + gpio_state_interfaces_.size());

  // add InterfaceDescriptions and create the StateInterfaces from the descriptions and add to
  // maps.
  for (const auto & description : unlisted_interface_descriptions)
  {
    auto name = description.get_name();
    unlisted_state_interfaces_.insert(std::make_pair(name, description));
    auto state_interface = std::make_shared<StateInterface>(description);
    impl_->hardware_states_.insert(std::make_pair(name, state_interface));
    unlisted_states_.push_back(state_interface);
    state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
  }

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    auto state_interface = std::make_shared<StateInterface>(descr);
    impl_->hardware_states_.insert(std::make_pair(name, state_interface));
    joint_states_.push_back(state_interface);
    state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
  }
  for (const auto & [name, descr] : sensor_state_interfaces_)
  {
    auto state_interface = std::make_shared<StateInterface>(descr);
    impl_->hardware_states_.insert(std::make_pair(name, state_interface));
    sensor_states_.push_back(state_interface);
    state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
  }
  for (const auto & [name, descr] : gpio_state_interfaces_)
  {
    auto state_interface = std::make_shared<StateInterface>(descr);
    impl_->hardware_states_.insert(std::make_pair(name, state_interface));
    gpio_states_.push_back(state_interface);
    state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
  }
  return state_interfaces;
}

std::vector<CommandInterface> HardwareComponentInterface::export_command_interfaces()
{
  // return empty vector by default. For backward compatibility we try calling
  // export_command_interfaces() and only when empty vector is returned call
  // on_export_command_interfaces()
  return {};
}

std::vector<hardware_interface::InterfaceDescription>
HardwareComponentInterface::export_unlisted_command_interface_descriptions()
{
  // return empty vector by default.
  return {};
}

std::vector<CommandInterface::SharedPtr> HardwareComponentInterface::on_export_command_interfaces()
{
  // import the unlisted interfaces
  std::vector<hardware_interface::InterfaceDescription> unlisted_interface_descriptions =
    export_unlisted_command_interface_descriptions();

  std::vector<CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(
    unlisted_interface_descriptions.size() + joint_command_interfaces_.size() +
    gpio_command_interfaces_.size());

  // add InterfaceDescriptions and create the CommandInterfaces from the descriptions and add to
  // maps.
  for (const auto & description : unlisted_interface_descriptions)
  {
    auto name = description.get_name();
    unlisted_command_interfaces_.insert(std::make_pair(name, description));
    auto command_interface = std::make_shared<CommandInterface>(description);
    impl_->hardware_commands_.insert(std::make_pair(name, command_interface));
    unlisted_commands_.push_back(command_interface);
    command_interfaces.push_back(command_interface);
  }

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    auto command_interface = std::make_shared<CommandInterface>(descr);
    impl_->hardware_commands_.insert(std::make_pair(name, command_interface));
    joint_commands_.push_back(command_interface);
    command_interfaces.push_back(command_interface);
  }

  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    auto command_interface = std::make_shared<CommandInterface>(descr);
    impl_->hardware_commands_.insert(std::make_pair(name, command_interface));
    gpio_commands_.push_back(command_interface);
    command_interfaces.push_back(command_interface);
  }
  return command_interfaces;
}

return_type HardwareComponentInterface::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return return_type::OK;
}

return_type HardwareComponentInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return return_type::OK;
}

HardwareComponentCycleStatus HardwareComponentInterface::trigger_read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  HardwareComponentCycleStatus status;
  status.result = return_type::ERROR;
  if (info_.is_async)
  {
    status.result = impl_->read_return_info_.load(std::memory_order_acquire);
    const auto read_exec_time = impl_->read_execution_time_.load(std::memory_order_acquire);
    if (read_exec_time.count() > 0)
    {
      status.execution_time = read_exec_time;
    }
    const auto result = async_handler_->trigger_async_callback(time, period);
    status.successful = result.first;
    if (!status.successful)
    {
      RCLCPP_WARN_EXPRESSION(
        get_logger(), info_.async_params.print_warnings,
        "Trigger read/write called while the previous async trigger is still in progress for "
        "hardware interface : '%s'. Failed to trigger read/write cycle!",
        info_.name.c_str());
      status.result = return_type::OK;
      return status;
    }
  }
  else
  {
    const auto start_time = std::chrono::steady_clock::now();
    status.successful = true;
    status.result = read(time, period);
    status.execution_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start_time);
  }
  return status;
}

HardwareComponentCycleStatus HardwareComponentInterface::trigger_write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  HardwareComponentCycleStatus status;
  status.result = return_type::ERROR;
  if (info_.is_async)
  {
    status.successful = true;
    const auto write_exec_time = impl_->write_execution_time_.load(std::memory_order_acquire);
    if (write_exec_time.count() > 0)
    {
      status.execution_time = write_exec_time;
    }
    status.result = impl_->write_return_info_.load(std::memory_order_acquire);
  }
  else
  {
    const auto start_time = std::chrono::steady_clock::now();
    status.successful = true;
    status.result = write(time, period);
    status.execution_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start_time);
  }
  return status;
}

return_type HardwareComponentInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return return_type::OK;
}

const std::string & HardwareComponentInterface::get_name() const { return info_.name; }

const std::string & HardwareComponentInterface::get_group_name() const { return info_.group; }

const rclcpp_lifecycle::State & HardwareComponentInterface::get_lifecycle_state() const
{
  return lifecycle_state_;
}

void HardwareComponentInterface::set_lifecycle_state(const rclcpp_lifecycle::State & new_state)
{
  lifecycle_state_ = new_state;
  impl_->lifecycle_id_cache_.store(new_state.id(), std::memory_order_release);
}

uint8_t HardwareComponentInterface::get_lifecycle_id() const
{
  return impl_->lifecycle_id_cache_.load(std::memory_order_acquire);
}

bool HardwareComponentInterface::has_state(const std::string & interface_name) const
{
  return impl_->hardware_states_.find(interface_name) != impl_->hardware_states_.end();
}

const StateInterface::SharedPtr & HardwareComponentInterface::get_state_interface_handle(
  const std::string & interface_name) const
{
  auto it = impl_->hardware_states_.find(interface_name);
  if (it == impl_->hardware_states_.end())
  {
    throw std::runtime_error(
      fmt::format(
        "The requested state interface not found: '{}' in hardware component: '{}'.",
        interface_name, info_.name));
  }
  return it->second;
}

bool HardwareComponentInterface::has_command(const std::string & interface_name) const
{
  return impl_->hardware_commands_.find(interface_name) != impl_->hardware_commands_.end();
}

const CommandInterface::SharedPtr & HardwareComponentInterface::get_command_interface_handle(
  const std::string & interface_name) const
{
  auto it = impl_->hardware_commands_.find(interface_name);
  if (it == impl_->hardware_commands_.end())
  {
    throw std::runtime_error(
      fmt::format(
        "The requested command interface not found: '{}' in hardware component: '{}'.",
        interface_name, info_.name));
  }
  return it->second;
}

rclcpp::Logger HardwareComponentInterface::get_logger() const { return impl_->logger_; }

rclcpp::Clock::SharedPtr HardwareComponentInterface::get_clock() const { return impl_->clock_; }

rclcpp::Node::SharedPtr HardwareComponentInterface::get_node() const
{
  return impl_->hardware_component_node_;
}

const HardwareInfo & HardwareComponentInterface::get_hardware_info() const { return info_; }

void HardwareComponentInterface::pause_async_operations()

{
  if (async_handler_)
  {
    async_handler_->pause_execution();
  }
}

void HardwareComponentInterface::prepare_for_activation()
{
  impl_->read_return_info_.store(return_type::OK, std::memory_order_release);
  impl_->read_execution_time_.store(std::chrono::nanoseconds::zero(), std::memory_order_release);
  impl_->write_return_info_.store(return_type::OK, std::memory_order_release);
  impl_->write_execution_time_.store(std::chrono::nanoseconds::zero(), std::memory_order_release);
}

void HardwareComponentInterface::enable_introspection(bool enable)
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

}  // namespace hardware_interface
