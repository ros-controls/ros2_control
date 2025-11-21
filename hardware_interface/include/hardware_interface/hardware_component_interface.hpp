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

#ifndef HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_
#define HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_

#include <fmt/compile.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/msg/hardware_status.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/introspection.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/trigger_type.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/version.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/async_function_handler.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace hardware_interface
{

static inline rclcpp::NodeOptions get_hardware_component_node_options()
{
  rclcpp::NodeOptions node_options;
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 21
  node_options.enable_logger_service(true);
#endif
  return node_options;
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Virtual base class for all hardware components (Actuators, Sensors, and Systems).
 *
 * This class provides the common structure and functionality for all hardware components,
 * including lifecycle management, interface handling, and asynchronous support. Hardware
 * plugins should inherit from one of its derivatives: ActuatorInterface, SensorInterface,
 * or SystemInterface.
 */
class HardwareComponentInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  HardwareComponentInterface()
  : lifecycle_state_(
      rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN)),
    logger_(rclcpp::get_logger("hardware_component_interface"))
  {
  }

  /// HardwareComponentInterface copy constructor is actively deleted.
  /**
   * Hardware interfaces have unique ownership and thus can't be copied in order to avoid
   * failed or simultaneous access to hardware.
   */
  HardwareComponentInterface(const HardwareComponentInterface & other) = delete;

  HardwareComponentInterface(HardwareComponentInterface && other) = delete;

  virtual ~HardwareComponentInterface() = default;

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] params  A struct of type HardwareComponentParams containing all necessary
   * parameters for initializing this specific hardware component,
   * including its HardwareInfo, a dedicated logger, a clock, and a
   * weak_ptr to the executor.
   * \warning The parsed executor should not be used to call `cancel()` or use blocking callbacks
   * such as `spin()`.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn init(const hardware_interface::HardwareComponentParams & params)
  {
    clock_ = params.clock;
    auto logger_copy = params.logger;
    logger_ = logger_copy.get_child(
      "hardware_component." + params.hardware_info.type + "." + params.hardware_info.name);
    info_ = params.hardware_info;
    if (params.hardware_info.is_async)
    {
      realtime_tools::AsyncFunctionHandlerParams async_thread_params;
      async_thread_params.thread_priority = info_.async_params.thread_priority;
      async_thread_params.scheduling_policy =
        realtime_tools::AsyncSchedulingPolicy(info_.async_params.scheduling_policy);
      async_thread_params.cpu_affinity_cores = info_.async_params.cpu_affinity_cores;
      async_thread_params.clock = params.clock;
      async_thread_params.logger = params.logger;
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
          read_return_info_.store(ret_read, std::memory_order_release);
          read_execution_time_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(read_end_time - read_start_time),
            std::memory_order_release);
          if (ret_read != return_type::OK)
          {
            return ret_read;
          }
          if (
            !is_sensor_type &&
            this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
          {
            const auto write_start_time = std::chrono::steady_clock::now();
            const auto ret_write = write(time, period);
            const auto write_end_time = std::chrono::steady_clock::now();
            write_return_info_.store(ret_write, std::memory_order_release);
            write_execution_time_.store(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                write_end_time - write_start_time),
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
      hardware_component_node_ = std::make_shared<rclcpp::Node>(
        node_name, params.node_namespace, get_hardware_component_node_options());
      locked_executor->add_node(hardware_component_node_->get_node_base_interface());
    }
    else
    {
      RCLCPP_WARN(
        params.logger,
        "Executor is not available during hardware component initialization for '%s'. Skipping "
        "node creation!",
        params.hardware_info.name.c_str());
    }

    double publish_rate = 0.0;
    auto it = info_.hardware_parameters.find("status_publish_rate");
    if (it != info_.hardware_parameters.end())
    {
      try
      {
        publish_rate = hardware_interface::stod(it->second);
      }
      catch (const std::invalid_argument &)
      {
        RCLCPP_WARN(
          get_logger(), "Invalid 'status_publish_rate' parameter. Using default %.1f Hz.",
          publish_rate);
      }
    }

    if (publish_rate == 0.0)
    {
      RCLCPP_INFO(
        get_logger(),
        "`status_publish_rate` is set to 0.0, hardware status publisher will not be created.");
    }
    else
    {
      control_msgs::msg::HardwareStatus status_msg_template;
      if (init_hardware_status_message(status_msg_template) != CallbackReturn::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "User-defined 'init_hardware_status_message' failed.");
        return CallbackReturn::ERROR;
      }

      if (!status_msg_template.hardware_device_states.empty())
      {
        if (!hardware_component_node_)
        {
          RCLCPP_WARN(
            get_logger(),
            "Hardware status message was configured, but no node is available for the publisher. "
            "Publisher will not be created.");
        }
        else
        {
          try
          {
            hardware_status_publisher_ =
              hardware_component_node_->create_publisher<control_msgs::msg::HardwareStatus>(
                "~/hardware_status", rclcpp::SystemDefaultsQoS());

            hardware_status_timer_ = hardware_component_node_->create_wall_timer(
              std::chrono::duration<double>(1.0 / publish_rate),
              [this]()
              {
                std::optional<control_msgs::msg::HardwareStatus> msg_to_publish_opt;
                hardware_status_box_.get(msg_to_publish_opt);

                if (msg_to_publish_opt.has_value() && hardware_status_publisher_)
                {
                  control_msgs::msg::HardwareStatus & msg = msg_to_publish_opt.value();
                  if (update_hardware_status_message(msg) != return_type::OK)
                  {
                    RCLCPP_WARN_THROTTLE(
                      get_logger(), *clock_, 1000,
                      "User's update_hardware_status_message() failed for '%s'.",
                      info_.name.c_str());
                    return;
                  }
                  msg.header.stamp = this->get_clock()->now();
                  hardware_status_publisher_->publish(msg);
                }
              });
            hardware_status_box_.set(std::make_optional(status_msg_template));
          }
          catch (const std::exception & e)
          {
            RCLCPP_ERROR(
              get_logger(), "Exception during publisher/timer setup for hardware status: %s",
              e.what());
            return CallbackReturn::ERROR;
          }
        }
      }
      else
      {
        RCLCPP_WARN(
          get_logger(),
          "`status_publish_rate` was set to a non-zero value, but no hardware status message was "
          "configured. Publisher will not be created. Are you sure "
          "init_hardware_status_message() is set up properly?");
      }
    }

    hardware_interface::HardwareComponentInterfaceParams interface_params;
    interface_params.hardware_info = info_;
    interface_params.executor = params.executor;
    return on_init(interface_params);
  };

  /// User-overridable method to configure the structure of the HardwareStatus message.
  /**
   * To enable status publishing, override this method to pre-allocate the message structure
   * and fill in static information like device IDs and interface names. This method is called
   * once during the non-realtime `init()` phase. If the `hardware_device_states` vector is
   * left empty, publishing will be disabled.
   *
   * \param[out] msg_template A reference to a HardwareStatus message to be configured.
   * \returns CallbackReturn::SUCCESS if configured successfully, CallbackReturn::ERROR on failure.
   */
  virtual CallbackReturn init_hardware_status_message(
    control_msgs::msg::HardwareStatus & /*msg_template*/)
  {
    // Default implementation does nothing, disabling the feature.
    return CallbackReturn::SUCCESS;
  }

  /// User-overridable method to fill the hardware status message with real-time data.
  /**
   * This real-time safe method is called by the framework within the `trigger_read()` loop.
   * Override this method to populate the `value` fields of the pre-allocated message with the
   * latest hardware states that were updated in your `read()` method.
   *
   * \param[in,out] msg The pre-allocated message to be filled with the latest values.
   * \returns return_type::OK on success, return_type::ERROR on failure.
   */
  virtual return_type update_hardware_status_message(control_msgs::msg::HardwareStatus & /*msg*/)
  {
    // Default implementation does nothing.
    return return_type::OK;
  }

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
   * \param[in] params  A struct of type hardware_interface::HardwareComponentInterfaceParams
   * containing all necessary parameters for initializing this specific hardware component,
   * specifically its HardwareInfo, and a weak_ptr to the executor.
   * \warning The parsed executor should not be used to call `cancel()` or use blocking callbacks
   * such as `spin()`.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
  {
    info_ = params.hardware_info;
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
  };

  /// Exports all state interfaces for this hardware interface.
  /**
   * Old way of exporting the StateInterfaces. If a empty vector is returned then
   * the on_export_state_interfaces() method is called. If a vector with StateInterfaces is returned
   * then the exporting of the StateInterfaces is only done with this function and the ownership is
   * transferred to the resource manager. The set_command(...), get_command(...), ..., can then not
   * be used.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
   */
  [[deprecated(
    "Replaced by vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() method. "
    "Exporting is handled by the Framework.")]] virtual std::vector<StateInterface>
  export_state_interfaces()
  {
    // return empty vector by default. For backward compatibility we try calling
    // export_state_interfaces() and only when empty vector is returned call
    // on_export_state_interfaces()
    return {};
  }

  /**
   * Override this method to export custom StateInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_state_interfaces_ map.
   *
   * \return vector of descriptions to the unlisted StateInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions()
  {
    // return empty vector by default.
    return {};
  }

  /**
   * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the interface.
   *
   * \return vector of shared pointers to the created and stored StateInterfaces
   */
  virtual std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces()
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
      hardware_states_.insert(std::make_pair(name, state_interface));
      unlisted_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      hardware_states_.insert(std::make_pair(name, state_interface));
      joint_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }
    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      hardware_states_.insert(std::make_pair(name, state_interface));
      sensor_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }
    for (const auto & [name, descr] : gpio_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      hardware_states_.insert(std::make_pair(name, state_interface));
      gpio_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }
    return state_interfaces;
  }

  /// Exports all command interfaces for this hardware interface.
  /**
   * Old way of exporting the CommandInterfaces. If a empty vector is returned then
   * the on_export_command_interfaces() method is called. If a vector with CommandInterfaces is
   * returned then the exporting of the CommandInterfaces is only done with this function and the
   * ownership is transferred to the resource manager. The set_command(...), get_command(...), ...,
   * can then not be used.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
   */
  [[deprecated(
    "Replaced by vector<CommandInterface::SharedPtr> on_export_command_interfaces() method. "
    "Exporting is handled by the Framework.")]] virtual std::vector<CommandInterface>
  export_command_interfaces()
  {
    // return empty vector by default. For backward compatibility we try calling
    // export_command_interfaces() and only when empty vector is returned call
    // on_export_command_interfaces()
    return {};
  }

  /**
   * Override this method to export custom CommandInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_command_interfaces_ map.
   *
   * \return vector of descriptions to the unlisted CommandInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions()
  {
    // return empty vector by default.
    return {};
  }

  /**
   * Default implementation for exporting the CommandInterfaces. The CommandInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * Actuator and System components should override this method. Sensor components can use the
   * default.
   *
   * \return vector of shared pointers to the created and stored CommandInterfaces
   */
  virtual std::vector<CommandInterface::SharedPtr> on_export_command_interfaces()
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
      hardware_commands_.insert(std::make_pair(name, command_interface));
      unlisted_commands_.push_back(command_interface);
      command_interfaces.push_back(command_interface);
    }

    for (const auto & [name, descr] : joint_command_interfaces_)
    {
      auto command_interface = std::make_shared<CommandInterface>(descr);
      hardware_commands_.insert(std::make_pair(name, command_interface));
      joint_commands_.push_back(command_interface);
      command_interfaces.push_back(command_interface);
    }

    for (const auto & [name, descr] : gpio_command_interfaces_)
    {
      auto command_interface = std::make_shared<CommandInterface>(descr);
      hardware_commands_.insert(std::make_pair(name, command_interface));
      gpio_commands_.push_back(command_interface);
      command_interfaces.push_back(command_interface);
    }
    return command_interfaces;
  }

  /// Prepare for a new command interface switch.
  /**
   * Prepare for any mode-switching required by the new command interface combination.
   *
   * \note This is a non-realtime evaluation of whether a set of command interface claims are
   * possible, and call to start preparing data structures for the upcoming switch that will occur.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return return_type::OK if the new command interface combination can be prepared (or) if the
   * interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  // Perform switching to the new command interface.
  /**
   * Perform the mode-switching for the new command interface combination.
   *
   * \note This is part of the realtime update loop, and should be fast.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return return_type::OK if the new command interface combination can be switched to (or) if the
   * interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  /// Triggers the read method synchronously or asynchronously depending on the HardwareInfo
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   * The method is called in the resource_manager's read loop
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  HardwareComponentCycleStatus trigger_read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    HardwareComponentCycleStatus status;
    status.result = return_type::ERROR;
    if (info_.is_async)
    {
      status.result = read_return_info_.load(std::memory_order_acquire);
      const auto read_exec_time = read_execution_time_.load(std::memory_order_acquire);
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

  /// Read the current state values from the hardware.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Triggers the write method synchronously or asynchronously depending on the HardwareInfo
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   * The method is called in the resource_manager's write loop
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  HardwareComponentCycleStatus trigger_write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    HardwareComponentCycleStatus status;
    status.result = return_type::ERROR;
    if (info_.is_async)
    {
      status.successful = true;
      const auto write_exec_time = write_execution_time_.load(std::memory_order_acquire);
      if (write_exec_time.count() > 0)
      {
        status.execution_time = write_exec_time;
      }
      status.result = write_return_info_.load(std::memory_order_acquire);
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

  /// Write the current command values to the hardware.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return return_type::OK;
  }

  /// Get name of the hardware.
  /**
   * \return name.
   */
  const std::string & get_name() const { return info_.name; }

  /// Get name of the hardware group to which it belongs to.
  /**
   * \return group name.
   */
  const std::string & get_group_name() const { return info_.group; }

  /// Get life-cycle state of the hardware.
  /**
   * \return state.
   */
  const rclcpp_lifecycle::State & get_lifecycle_state() const { return lifecycle_state_; }

  /// Set life-cycle state of the hardware.
  /**
   * \return state.
   */
  void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)
  {
    lifecycle_state_ = new_state;
  }

  /// Does the state interface exist?
  /**
   * \param[in] interface_name The name of the state interface.
   * \return true if the state interface exists, false otherwise.
   */
  bool has_state(const std::string & interface_name) const
  {
    return hardware_states_.find(interface_name) != hardware_states_.end();
  }

  /// Set the value of a state interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param[in] interface_name The name of the state interface to access.
   * \param[in] value The value to store.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface.
   */
  template <typename T>
  void set_state(const std::string & interface_name, const T & value)
  {
    auto it = hardware_states_.find(interface_name);
    if (it == hardware_states_.end())
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE(
            "State interface not found: {} in hardware component: {}. "
            "This should not happen."),
          interface_name, info_.name));
    }
    auto & handle = it->second;
    std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
    std::ignore = handle->set_value(lock, value);
  }

  /// Get the value from a state interface.
  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_name The name of the state interface to access.
   * \return The value obtained from the interface.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface or its stored value.
   */
  template <typename T = double>
  T get_state(const std::string & interface_name) const
  {
    auto it = hardware_states_.find(interface_name);
    if (it == hardware_states_.end())
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE(
            "State interface not found: {} in hardware component: {}. "
            "This should not happen."),
          interface_name, info_.name));
    }
    auto & handle = it->second;
    std::shared_lock<std::shared_mutex> lock(handle->get_mutex());
    const auto opt_value = handle->get_optional<T>(lock);
    if (!opt_value)
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE("Failed to get state value from interface: {}. This should not happen."),
          interface_name));
    }
    return opt_value.value();
  }

  /// Does the command interface exist?
  /**
   * \param[in] interface_name The name of the command interface.
   * \return true if the command interface exists, false otherwise.
   */
  bool has_command(const std::string & interface_name) const
  {
    return hardware_commands_.find(interface_name) != hardware_commands_.end();
  }

  /// Set the value of a command interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param interface_name The name of the command
   * interface to access.
   * \param value The value to store.
   * \throws This method throws a runtime error if it
   * cannot access the command interface.
   */
  template <typename T>
  void set_command(const std::string & interface_name, const T & value)
  {
    auto it = hardware_commands_.find(interface_name);
    if (it == hardware_commands_.end())
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE(
            "Command interface not found: {} in hardware component: {}. "
            "This should not happen."),
          interface_name, info_.name));
    }
    auto & handle = it->second;
    std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
    std::ignore = handle->set_value(lock, value);
  }

  ///  Get the value from a command interface.
  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_name The name of the command interface to access.
   * \return The value obtained from the interface.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the command interface or its stored value.
   */
  template <typename T = double>
  T get_command(const std::string & interface_name) const
  {
    auto it = hardware_commands_.find(interface_name);
    if (it == hardware_commands_.end())
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE(
            "Command interface not found: {} in hardware component: {}. "
            "This should not happen."),
          interface_name, info_.name));
    }
    auto & handle = it->second;
    std::shared_lock<std::shared_mutex> lock(handle->get_mutex());
    const auto opt_value = handle->get_optional<T>(lock);
    if (!opt_value)
    {
      throw std::runtime_error(
        fmt::format(
          FMT_COMPILE("Failed to get command value from interface: {}. This should not happen."),
          interface_name));
    }
    return opt_value.value();
  }

  /// Get the logger of the HardwareComponentInterface.
  /**
   * \return logger of the HardwareComponentInterface.
   */
  rclcpp::Logger get_logger() const { return logger_; }

  /// Get the clock
  /**
   * \return clock that is shared with the controller manager
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  /// Get the default node of the HardwareComponentInterface.
  /**
   * \return node of the HardwareComponentInterface.
   */
  rclcpp::Node::SharedPtr get_node() const { return hardware_component_node_; }

  /// Get the hardware info of the HardwareComponentInterface.
  /**
   * \return hardware info of the HardwareComponentInterface.
   */
  const HardwareInfo & get_hardware_info() const { return info_; }

  /// Pause any asynchronous operations.
  /**
   * This method is called to pause any ongoing asynchronous operations, such as read/write cycles.
   * It is typically used during lifecycle transitions or when the hardware needs to be paused.
   */
  void pause_async_operations()
  {
    if (async_handler_)
    {
      async_handler_->pause_execution();
    }
  }

  /// Prepare for the activation of the hardware.
  /**
   * This method is called before the hardware is activated by the resource manager.
   */
  void prepare_for_activation()
  {
    read_return_info_.store(return_type::OK, std::memory_order_release);
    read_execution_time_.store(std::chrono::nanoseconds::zero(), std::memory_order_release);
    write_return_info_.store(return_type::OK, std::memory_order_release);
    write_execution_time_.store(std::chrono::nanoseconds::zero(), std::memory_order_release);
  }

  /// Enable or disable introspection of the hardware.
  /**
   * \param[in] enable Enable introspection if true, disable otherwise.
   */
  void enable_introspection(bool enable)
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

protected:
  HardwareInfo info_;
  // interface names to InterfaceDescription
  std::unordered_map<std::string, InterfaceDescription> joint_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> joint_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> sensor_state_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> gpio_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> gpio_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> unlisted_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> unlisted_command_interfaces_;

  rclcpp_lifecycle::State lifecycle_state_;
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<return_type>> async_handler_;

  // Exported Command- and StateInterfaces in order they are listed in the hardware description.
  std::vector<StateInterface::SharedPtr> joint_states_;
  std::vector<CommandInterface::SharedPtr> joint_commands_;

  std::vector<StateInterface::SharedPtr> sensor_states_;

  std::vector<StateInterface::SharedPtr> gpio_states_;
  std::vector<CommandInterface::SharedPtr> gpio_commands_;

  std::vector<StateInterface::SharedPtr> unlisted_states_;
  std::vector<CommandInterface::SharedPtr> unlisted_commands_;

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  rclcpp::Node::SharedPtr hardware_component_node_ = nullptr;
  // interface names to Handle accessed through getters/setters
  std::unordered_map<std::string, StateInterface::SharedPtr> hardware_states_;
  std::unordered_map<std::string, CommandInterface::SharedPtr> hardware_commands_;
  std::atomic<return_type> read_return_info_ = return_type::OK;
  std::atomic<std::chrono::nanoseconds> read_execution_time_ = std::chrono::nanoseconds::zero();
  std::atomic<return_type> write_return_info_ = return_type::OK;
  std::atomic<std::chrono::nanoseconds> write_execution_time_ = std::chrono::nanoseconds::zero();

protected:
  pal_statistics::RegistrationsRAII stats_registrations_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::HardwareStatus>> hardware_status_publisher_;
  realtime_tools::RealtimeThreadSafeBox<std::optional<control_msgs::msg::HardwareStatus>>
    hardware_status_box_;
  rclcpp::TimerBase::SharedPtr hardware_status_timer_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_
