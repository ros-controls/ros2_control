// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "hardware_interface/resource_manager.hpp"

#include <fmt/compile.h>

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "joint_limits/joint_limits_helpers.hpp"
#include "joint_limits/joint_saturation_limiter.hpp"
#include "joint_limits/joint_soft_limiter.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

namespace hardware_interface
{
auto trigger_and_print_hardware_state_transition =
  [](
    auto transition, const std::string transition_name, const std::string & hardware_name,
    const lifecycle_msgs::msg::State::_id_type & target_state)
{
  RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "'%s' hardware '%s' ", transition_name.c_str(), hardware_name.c_str());

  const rclcpp_lifecycle::State new_state = transition();

  bool result = new_state.id() == target_state;

  if (result)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Successful '%s' of hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  else
  {
    RCUTILS_LOG_ERROR_NAMED(
      "resource_manager", "Failed to '%s' hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  return result;
};

std::string interfaces_to_string(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::stringstream ss;
  ss << "Start interfaces: " << std::endl << "[" << std::endl;
  for (const auto & start_if : start_interfaces)
  {
    ss << "  " << start_if << std::endl;
  }
  ss << "]" << std::endl;
  ss << "Stop interfaces: " << std::endl << "[" << std::endl;
  for (const auto & stop_if : stop_interfaces)
  {
    ss << "  " << stop_if << std::endl;
  }
  ss << "]" << std::endl;
  return ss.str();
};

void find_common_hardware_interfaces(
  const std::vector<std::string> & hw_command_itfs,
  const std::vector<std::string> & start_stop_interfaces_list,
  std::vector<std::string> & hw_interfaces)
{
  hw_interfaces.clear();

  // decide which input vector is shorter.
  const auto & shorter_vec = hw_command_itfs.size() < start_stop_interfaces_list.size()
                               ? hw_command_itfs
                               : start_stop_interfaces_list;
  const auto & longer_vec =
    &shorter_vec == &hw_command_itfs ? start_stop_interfaces_list : hw_command_itfs;

  // reserve exactly the worst-case result size (all of the smaller one).
  hw_interfaces.reserve(shorter_vec.size());

  // build a hash set from the smaller vector.
  std::unordered_set<std::string> lookup(shorter_vec.begin(), shorter_vec.end());

  // iterate through the larger vector; test membership in constant time.
  for (const auto & name : longer_vec)
  {
    if (lookup.find(name) != lookup.end())
    {
      hw_interfaces.push_back(name);
    }
  }
}

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * actuator_interface_name = "hardware_interface::ActuatorInterface";
  static constexpr const char * sensor_interface_name = "hardware_interface::SensorInterface";
  static constexpr const char * system_interface_name = "hardware_interface::SystemInterface";

public:
  // TODO(VX792): Change this when HW ifs get their own update rate,
  // because the ResourceStorage really shouldn't know about the cm's parameters
  explicit ResourceStorage(
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface)
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name),
    rm_logger_(rclcpp::get_logger("resource_manager"))
  {
    if (!clock_interface)
    {
      throw std::invalid_argument(
        "Clock interface is nullptr. ResourceManager needs a valid clock interface.");
    }
    rm_clock_ = clock_interface->get_clock();
    if (logger_interface)
    {
      rm_logger_ = logger_interface->get_logger().get_child("resource_manager");
    }
  }

  explicit ResourceStorage(rclcpp::Clock::SharedPtr clock_interface, rclcpp::Logger logger)
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name),
    rm_clock_(clock_interface),
    rm_logger_(logger)
  {
    if (!rm_clock_)
    {
      throw std::invalid_argument("Clock is nullptr. ResourceManager needs a valid clock.");
    }
  }

  template <class HardwareT, class HardwareInterfaceT>
  [[nodiscard]] bool load_hardware(
    const HardwareInfo & hardware_info, pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    bool is_loaded = false;
    try
    {
      RCLCPP_INFO(get_logger(), "Loading hardware '%s' ", hardware_info.name.c_str());
      // hardware_plugin_name has to match class name in plugin xml description
      auto interface = std::unique_ptr<HardwareInterfaceT>(
        loader.createUnmanagedInstance(hardware_info.hardware_plugin_name));
      if (interface)
      {
        RCLCPP_INFO(
          get_logger(), "Loaded hardware '%s' from plugin '%s'", hardware_info.name.c_str(),
          hardware_info.hardware_plugin_name.c_str());
        HardwareT hardware(std::move(interface));
        container.emplace_back(std::move(hardware));
        // initialize static data about hardware component to reduce later calls
        HardwareComponentInfo component_info;
        component_info.name = hardware_info.name;
        component_info.type = hardware_info.type;
        component_info.group = hardware_info.group;
        component_info.rw_rate = hardware_info.rw_rate;
        component_info.plugin_name = hardware_info.hardware_plugin_name;
        component_info.is_async = hardware_info.is_async;
        component_info.read_statistics = std::make_shared<HardwareComponentStatisticsData>();

        // if the type of the hardware is sensor then don't initialize the write statistics
        if (hardware_info.type != "sensor")
        {
          component_info.write_statistics = std::make_shared<HardwareComponentStatisticsData>();
        }

        hardware_info_map_.insert(std::make_pair(component_info.name, component_info));
        hw_group_state_.insert(std::make_pair(component_info.group, return_type::OK));
        hardware_used_by_controllers_.insert(
          std::make_pair(component_info.name, std::vector<std::string>()));
        is_loaded = true;
      }
      else
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to load hardware '%s' from plugin '%s'", hardware_info.name.c_str(),
          hardware_info.hardware_plugin_name.c_str());
      }
    }
    catch (const pluginlib::PluginlibException & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Caught exception of type : %s while loading hardware: %s", typeid(ex).name(),
        ex.what());
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while loading hardware '%s': %s",
        typeid(ex).name(), hardware_info.name.c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while loading hardware '%s'",
        hardware_info.name.c_str());
    }
    return is_loaded;
  }

  template <class HardwareT>
  bool initialize_hardware(const HardwareInfo & hardware_info, HardwareT & hardware)
  {
    RCLCPP_INFO(get_logger(), "Initialize hardware '%s' ", hardware_info.name.c_str());

    bool result = false;
    try
    {
      const rclcpp_lifecycle::State new_state =
        hardware.initialize(hardware_info, rm_logger_, rm_clock_);
      result = new_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

      if (result)
      {
        RCLCPP_INFO(
          get_logger(), "Successful initialization of hardware '%s'", hardware_info.name.c_str());
      }
      else
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to initialize hardware '%s'", hardware_info.name.c_str());
      }
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while initializing hardware '%s': %s",
        typeid(ex).name(), hardware_info.name.c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while initializing hardware '%s'",
        hardware_info.name.c_str());
    }

    return result;
  }

  template <class HardwareT>
  bool configure_hardware(HardwareT & hardware)
  {
    bool result = false;
    try
    {
      result = trigger_and_print_hardware_state_transition(
        std::bind(&HardwareT::configure, &hardware), "configure", hardware.get_name(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while configuring hardware '%s': %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while configuring hardware '%s'",
        hardware.get_name().c_str());
    }

    if (result)
    {
      // TODO(destogl): is it better to check here if previous state was unconfigured instead of
      // checking if each state already exists? Or we should somehow know that transition has
      // happened and only then trigger this part of the code?
      // On the other side this part of the code should never be executed in real-time critical
      // thread, so it could be also OK as it is...
      for (const auto & interface : hardware_info_map_[hardware.get_name()].state_interfaces)
      {
        // add all state interfaces to available list
        auto found_it = std::find(
          available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

        if (found_it == available_state_interfaces_.end())
        {
          available_state_interfaces_.emplace_back(interface);
          RCLCPP_DEBUG(
            get_logger(), "(hardware '%s'): '%s' state interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCLCPP_WARN(
            get_logger(),
            "(hardware '%s'): '%s' state interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }

      // add command interfaces to available list
      for (const auto & interface : hardware_info_map_[hardware.get_name()].command_interfaces)
      {
        // TODO(destogl): check if interface should be available on configure
        auto found_it = std::find(
          available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

        if (found_it == available_command_interfaces_.end())
        {
          available_command_interfaces_.emplace_back(interface);
          RCLCPP_DEBUG(
            get_logger(), "(hardware '%s'): '%s' command interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCLCPP_WARN(
            get_logger(),
            "(hardware '%s'): '%s' command interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }
    }
    if (!hardware.get_group_name().empty())
    {
      hw_group_state_[hardware.get_group_name()] = return_type::OK;
    }
    return result;
  }

  void remove_all_hardware_interfaces_from_available_list(const std::string & hardware_name)
  {
    // remove all command interfaces from available list
    for (const auto & interface : hardware_info_map_[hardware_name].command_interfaces)
    {
      auto found_it = std::find(
        available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

      if (found_it != available_command_interfaces_.end())
      {
        available_command_interfaces_.erase(found_it);
        RCLCPP_DEBUG(
          get_logger(), "(hardware '%s'): '%s' command interface removed from available list",
          hardware_name.c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCLCPP_WARN(
          get_logger(),
          "(hardware '%s'): '%s' command interface not in available list. "
          "This should not happen (hint: multiple cleanup calls).",
          hardware_name.c_str(), interface.c_str());
      }
    }
    // remove all state interfaces from available list
    for (const auto & interface : hardware_info_map_[hardware_name].state_interfaces)
    {
      auto found_it = std::find(
        available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

      if (found_it != available_state_interfaces_.end())
      {
        available_state_interfaces_.erase(found_it);
        RCLCPP_DEBUG(
          get_logger(), "(hardware '%s'): '%s' state interface removed from available list",
          hardware_name.c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCLCPP_WARN(
          get_logger(),
          "(hardware '%s'): '%s' state interface not in available list. "
          "This should not happen (hint: multiple cleanup calls).",
          hardware_name.c_str(), interface.c_str());
      }
    }
  }

  template <class HardwareT>
  bool cleanup_hardware(HardwareT & hardware)
  {
    bool result = false;
    try
    {
      result = trigger_and_print_hardware_state_transition(
        std::bind(&HardwareT::cleanup, &hardware), "cleanup", hardware.get_name(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while cleaning up hardware '%s': %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while cleaning up hardware '%s'",
        hardware.get_name().c_str());
    }

    if (result)
    {
      remove_all_hardware_interfaces_from_available_list(hardware.get_name());
    }
    if (!hardware.get_group_name().empty())
    {
      hw_group_state_[hardware.get_group_name()] = return_type::OK;
    }
    return result;
  }

  template <class HardwareT>
  bool shutdown_hardware(HardwareT & hardware)
  {
    bool result = false;
    try
    {
      result = trigger_and_print_hardware_state_transition(
        std::bind(&HardwareT::shutdown, &hardware), "shutdown", hardware.get_name(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while shutting down hardware '%s': %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while shutting down hardware '%s'",
        hardware.get_name().c_str());
    }

    if (result)
    {
      remove_all_hardware_interfaces_from_available_list(hardware.get_name());
      // TODO(destogl): change this - deimport all things if there is there are interfaces there
      // deimport_non_movement_command_interfaces(hardware);
      // deimport_state_interfaces(hardware);
      // use remove_command_interfaces(hardware);
      if (!hardware.get_group_name().empty())
      {
        hw_group_state_[hardware.get_group_name()] = return_type::OK;
      }
    }
    return result;
  }

  template <class HardwareT>
  bool activate_hardware(HardwareT & hardware)
  {
    bool result = false;
    try
    {
      result = trigger_and_print_hardware_state_transition(
        std::bind(&HardwareT::activate, &hardware), "activate", hardware.get_name(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while activating hardware '%s': %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while activating hardware '%s'",
        hardware.get_name().c_str());
    }
    return result;
  }

  template <class HardwareT>
  bool deactivate_hardware(HardwareT & hardware)
  {
    bool result = false;
    try
    {
      result = trigger_and_print_hardware_state_transition(
        std::bind(&HardwareT::deactivate, &hardware), "deactivate", hardware.get_name(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception of type : %s occurred while deactivating hardware '%s': %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(), "Unknown exception occurred while deactivating hardware '%s'",
        hardware.get_name().c_str());
    }

    if (result)
    {
      // TODO(destogl): make all command interfaces unavailable that should be present only
      // when active (currently are all available) also at inactive
    }
    return result;
  }

  template <class HardwareT>
  bool set_component_state(HardwareT & hardware, const rclcpp_lifecycle::State & target_state)
  {
    using lifecycle_msgs::msg::State;

    bool result = false;

    switch (target_state.id())
    {
      case State::PRIMARY_STATE_UNCONFIGURED:
        switch (hardware.get_lifecycle_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = true;
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = cleanup_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            if (result)
            {
              result = cleanup_hardware(hardware);
            }
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCLCPP_WARN(
              get_logger(), "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_INACTIVE:
        switch (hardware.get_lifecycle_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCLCPP_WARN(
              get_logger(), "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_ACTIVE:
        switch (hardware.get_lifecycle_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            if (result)
            {
              result = activate_hardware(hardware);
            }
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = activate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCLCPP_WARN(
              get_logger(), "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_FINALIZED:
        switch (hardware.get_lifecycle_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware) && shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = true;
            break;
        }
        break;
    }
    if (on_component_state_switch_callback_)
    {
      on_component_state_switch_callback_();
    }

    return result;
  }

  template <class HardwareT>
  void import_state_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_state_interfaces();
    const auto interface_names = add_state_interfaces(interfaces);

    RCLCPP_WARN_EXPRESSION(
      get_logger(), interface_names.empty(),
      "Importing state interfaces for the hardware '%s' returned no state interfaces.",
      hardware.get_name().c_str());
    hardware_info_map_[hardware.get_name()].state_interfaces = interface_names;
    available_state_interfaces_.reserve(
      available_state_interfaces_.capacity() + interface_names.size());
  }

  void insert_command_interface(const CommandInterface::SharedPtr command_interface)
  {
    const auto [it, success] = command_interface_map_.insert(
      std::make_pair(command_interface->get_name(), command_interface));
    if (!success)
    {
      const std::string msg = fmt::format(
        FMT_COMPILE(
          "ResourceStorage: Tried to insert CommandInterface with already existing key. "
          "Insert[{}]"),
        command_interface->get_name());
      throw std::runtime_error(msg);
    }
    command_interface->registerIntrospection();
  }

  // BEGIN (Handle export change): for backward compatibility, can be removed if
  // export_command_interfaces() method is removed
  void insert_command_interface(CommandInterface && command_interface)
  {
    std::string key = command_interface.get_name();
    const auto [it, success] = command_interface_map_.emplace(
      std::make_pair(key, std::make_shared<CommandInterface>(std::move(command_interface))));
    if (!success)
    {
      const std::string msg = fmt::format(
        FMT_COMPILE(
          "ResourceStorage: Tried to insert CommandInterface with already existing key. "
          "Insert[{}]"),
        key);
      throw std::runtime_error(msg);
    }
  }
  // END: for backward compatibility

  template <class HardwareT>
  void import_command_interfaces(HardwareT & hardware)
  {
    try
    {
      auto interfaces = hardware.export_command_interfaces();
      hardware_info_map_[hardware.get_name()].command_interfaces =
        add_command_interfaces(interfaces);
      start_interfaces_buffer_.reserve(start_interfaces_buffer_.capacity() + interfaces.size());
      stop_interfaces_buffer_.reserve(stop_interfaces_buffer_.capacity() + interfaces.size());
      // TODO(Manuel) END: for backward compatibility
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Exception of type : %s occurred while importing command interfaces for the hardware '%s' "
        ": %s",
        typeid(ex).name(), hardware.get_name().c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown exception occurred while importing command interfaces for the hardware '%s'",
        hardware.get_name().c_str());
    }
  }

  void import_joint_limiters(const std::vector<HardwareInfo> & hardware_infos)
  {
    for (const auto & hw_info : hardware_infos)
    {
      for (const auto & [joint_name, limits] : hw_info.limits)
      {
        std::vector<joint_limits::SoftJointLimits> soft_limits;
        const std::vector<joint_limits::JointLimits> hard_limits{limits};
        joint_limits::JointInterfacesCommandLimiterData data;
        data.joint_name = joint_name;
        limiters_data_.insert({joint_name, data});
        // If the joint limits is found in the softlimits, then extract it
        if (hw_info.soft_limits.find(joint_name) != hw_info.soft_limits.end())
        {
          soft_limits = {hw_info.soft_limits.at(joint_name)};
          RCLCPP_INFO(
            get_logger(), "Using SoftJointLimiter for joint '%s' in hardware '%s' : '%s'",
            joint_name.c_str(), hw_info.name.c_str(), soft_limits[0].to_string().c_str());
        }
        else
        {
          RCLCPP_INFO(
            get_logger(), "Using JointLimiter for joint '%s' in hardware '%s' : '%s'",
            joint_name.c_str(), hw_info.name.c_str(), limits.to_string().c_str());
        }
        std::unique_ptr<
          joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>>
          limits_interface;
        if (soft_limits.empty())
        {
          RCLCPP_INFO(
            get_logger(), "Creating JointSaturationLimiter for joint '%s' in hardware '%s'",
            joint_name.c_str(), hw_info.name.c_str());
          limits_interface = std::make_unique<
            joint_limits::JointSaturationLimiter<joint_limits::JointControlInterfacesData>>();
        }
        else
        {
          RCLCPP_INFO(
            get_logger(), "Creating JointSoftLimiter for joint '%s' in hardware '%s'",
            joint_name.c_str(), hw_info.name.c_str());
          limits_interface = std::make_unique<joint_limits::JointSoftLimiter>();
        }
        limits_interface->init({joint_name}, hard_limits, soft_limits, nullptr, nullptr);
        joint_limiters_interface_[hw_info.name].insert({joint_name, std::move(limits_interface)});
      }
    }
  }

  template <typename T>
  void update_joint_limiters_data(
    const std::string & joint_name, const std::map<std::string, T> & interface_map,
    joint_limits::JointControlInterfacesData & data, bool is_command_itf = false)
  {
    data.joint_name = joint_name;

    const auto fill_interface_data =
      [&](const std::string & interface_type, std::optional<double> & value)
    {
      const std::string interface_name =
        fmt::format(FMT_COMPILE("{}/{}"), joint_name, interface_type);
      if (interface_map.find(interface_name) != interface_map.end())
      {
        // If the command interface is not claimed, then the value is not set (or) if the
        // interface doesn't exist, then value is not set
        if (
          is_command_itf && (claimed_command_interface_map_.count(interface_name) == 0 ||
                             !claimed_command_interface_map_.at(interface_name)))
        {
          value = std::nullopt;
        }
        else
        {
          auto itf_handle = interface_map.at(interface_name);
          std::shared_lock<std::shared_mutex> lock(itf_handle->get_mutex());
          value = itf_handle->get_optional(lock).value();
        }
      }
    };
    // update the actual data of the limiters
    fill_interface_data(hardware_interface::HW_IF_POSITION, data.position);
    fill_interface_data(hardware_interface::HW_IF_VELOCITY, data.velocity);
    fill_interface_data(hardware_interface::HW_IF_EFFORT, data.effort);
    fill_interface_data(hardware_interface::HW_IF_ACCELERATION, data.acceleration);
  }

  template <typename T>
  void update_joint_limiters_commands(
    const joint_limits::JointControlInterfacesData & limited_command,
    std::map<std::string, T> & interface_map)
  {
    const auto set_interface_command =
      [&](const std::string & interface_type, const std::optional<double> & data)
    {
      const std::string interface_name =
        fmt::format(FMT_COMPILE("{}/{}"), limited_command.joint_name, interface_type);
      if (data.has_value() && interface_map.find(interface_name) != interface_map.end())
      {
        auto itf_handle = interface_map.at(interface_name);
        std::unique_lock<std::shared_mutex> lock(itf_handle->get_mutex());
        std::ignore = itf_handle->set_value(lock, data.value());
      }
    };
    // update the command data of the limiters
    set_interface_command(hardware_interface::HW_IF_POSITION, limited_command.position);
    set_interface_command(hardware_interface::HW_IF_VELOCITY, limited_command.velocity);
    set_interface_command(hardware_interface::HW_IF_EFFORT, limited_command.effort);
    set_interface_command(hardware_interface::HW_IF_ACCELERATION, limited_command.acceleration);
  }

  void update_joint_limiters_data(joint_limits::JointInterfacesCommandLimiterData & data)
  {
    update_joint_limiters_data(data.joint_name, state_interface_map_, data.actual);
    update_joint_limiters_data(data.joint_name, command_interface_map_, data.command, true);
    data.limited = data.command;
  }

  /// enforce the command limits for a specific joint
  /**
   * @param joint_name name of the joint to enforce the command limits
   * @param period time period of the command
   * @return true if the command interfaces are out of limits and the limits are enforced
   * @return false if the command interfaces values are within limits
   */
  bool enforce_command_limits(const std::string & joint_name, const rclcpp::Duration & period)
  {
    bool enforce_result = false;
    for (auto & [hw_name, limiters] : joint_limiters_interface_)
    {
      if (limiters.find(joint_name) != limiters.end())
      {
        joint_limits::JointInterfacesCommandLimiterData & data = limiters_data_[joint_name];
        update_joint_limiters_data(data);
        enforce_result = limiters[joint_name]->enforce(data.actual, data.limited, period);
        if (enforce_result)
        {
          update_joint_limiters_commands(data.limited, command_interface_map_);
        }
      }
    }
    return enforce_result;
  }

  std::string add_state_interface(StateInterface::ConstSharedPtr interface)
  {
    auto interface_name = interface->get_name();
    const auto [it, success] = state_interface_map_.emplace(interface_name, interface);
    if (!success)
    {
      const std::string msg = fmt::format(
        FMT_COMPILE(
          "ResourceStorage: Tried to insert StateInterface with already existing key. Insert[{}]"),
        interface->get_name());
      throw std::runtime_error(msg);
    }
    interface->registerIntrospection();
    return interface_name;
  }
  /// Adds exported state interfaces into internal storage.
  /**
   * Adds state interfaces to the internal storage. State interfaces exported from hardware or
   * chainable controllers are moved to the map with name-interface pairs and available list's
   * size is increased to reserve storage when interface change theirs status in real-time
   * control loop.
   *
   * \param[interfaces] list of state interface to add into storage.
   * \returns list of interface names that are added into internal storage. The output is used to
   * avoid additional iterations to cache interface names, e.g., for initializing info structures.
   */
  std::vector<std::string> add_state_interfaces(
    std::vector<StateInterface::ConstSharedPtr> & interfaces)
  {
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto & interface : interfaces)
    {
      try
      {
        interface_names.push_back(add_state_interface(interface));
      }
      // We don't want to crash during runtime because a StateInterface could not be added
      catch (const std::exception & e)
      {
        RCLCPP_WARN(
          get_logger(), "Exception occurred while importing state interfaces: %s", e.what());
      }
    }
    available_state_interfaces_.reserve(
      available_state_interfaces_.capacity() + interface_names.size());

    return interface_names;
  }

  /// Removes state interfaces from internal storage.
  /**
   * State interface are removed from the maps with theirs storage and their claimed status.
   *
   * \param[interface_names] list of state interface names to remove from storage.
   */
  void remove_state_interfaces(const std::vector<std::string> & interface_names)
  {
    for (const auto & interface : interface_names)
    {
      state_interface_map_[interface]->unregisterIntrospection();
      state_interface_map_.erase(interface);
    }
  }

  /// Adds exported command interfaces into internal storage.
  /**
   * Add command interfaces to the internal storage. Command interfaces exported from hardware or
   * chainable controllers are moved to the map with name-interface pairs, the interface names are
   * added to the claimed map and available list's size is increased to reserve storage when
   * interface change theirs status in real-time control loop.
   *
   * \param[interfaces] list of command interface to add into storage.
   * \returns list of interface names that are added into internal storage. The output is used to
   * avoid additional iterations to cache interface names, e.g., for initializing info structures.
   */
  std::vector<std::string> add_command_interfaces(std::vector<CommandInterface> & interfaces)
  {
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto & interface : interfaces)
    {
      auto key = interface.get_name();
      insert_command_interface(std::move(interface));
      claimed_command_interface_map_.emplace(std::make_pair(key, false));
      interface_names.push_back(key);
    }
    available_command_interfaces_.reserve(
      available_command_interfaces_.capacity() + interface_names.size());

    return interface_names;
  }

  std::vector<std::string> add_command_interfaces(
    const std::vector<CommandInterface::SharedPtr> & interfaces)
  {
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (const auto & interface : interfaces)
    {
      auto key = interface->get_name();
      bind_command_limiter_to_interface(interface);
      insert_command_interface(interface);
      claimed_command_interface_map_.emplace(std::make_pair(key, false));
      interface_names.push_back(key);
    }
    available_command_interfaces_.reserve(
      available_command_interfaces_.capacity() + interface_names.size());

    return interface_names;
  }

  /// Binds the command limiter enforcement to the command interfaces.
  /**
   * Binds the enforcement of the command limits to the command interfaces. The enforcement is
   * triggered by the command interfaces when the command is set.
   * If the interface prefix is a joint name, then the limit enforcement callback is added to the
   * command interface.
   *
   * \param[interface] command interface to bind the enforcement.
   */
  void bind_command_limiter_to_interface(CommandInterface::SharedPtr interface)
  {
    if (interface)
    {
      for (auto & [hw_name, limiters] : joint_limiters_interface_)
      {
        // If the prefix is a joint name, then bind the limiter to the command interface
        if (limiters.find(interface->get_prefix_name()) != limiters.end())
        {
          const std::string & joint_name = interface->get_prefix_name();
          const rclcpp::Duration desired_period =
            rclcpp::Duration::from_seconds(1.0 / cm_update_rate_);
          const std::string & interface_name = interface->get_interface_name();
          const std::vector<std::string> supported_interfaces = {
            hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_ACCELERATION};
          if (
            std::find(supported_interfaces.begin(), supported_interfaces.end(), interface_name) ==
            supported_interfaces.end())
          {
            RCLCPP_DEBUG(
              get_logger(), "Command interface '%s' is not supported for enforcing limits",
              interface_name.c_str());
            continue;
          }
          const auto limiter_fn = [this, joint_name, interface_name, desired_period, &limiters](
                                    double value, bool & is_limited) -> double
          {
            is_limited = false;
            joint_limits::JointInterfacesCommandLimiterData data;
            data.joint_name = joint_name;
            update_joint_limiters_data(data.joint_name, state_interface_map_, data.actual);
            if (interface_name == hardware_interface::HW_IF_POSITION)
            {
              data.command.position = value;
            }
            else if (interface_name == hardware_interface::HW_IF_VELOCITY)
            {
              data.command.velocity = value;
            }
            else if (interface_name == hardware_interface::HW_IF_EFFORT)
            {
              data.command.effort = value;
            }
            else if (interface_name == hardware_interface::HW_IF_ACCELERATION)
            {
              data.command.acceleration = value;
            }
            else
            {
              return value;
            }
            data.limited = data.command;
            is_limited = limiters[joint_name]->enforce(data.actual, data.limited, desired_period);
            if (is_limited)
            {
              RCLCPP_ERROR_THROTTLE(
                get_logger(), *rm_clock_, 1000,
                "Command of at least one joint is out of limits (throttled log). %s with desired "
                "period : %f sec.",
                data.to_string().c_str(), desired_period.seconds());
            }
            if (
              interface_name == hardware_interface::HW_IF_POSITION &&
              data.limited.position.has_value())
            {
              return data.limited.position.value();
            }
            else if (
              interface_name == hardware_interface::HW_IF_VELOCITY &&
              data.limited.velocity.has_value())
            {
              return data.limited.velocity.value();
            }
            else if (
              interface_name == hardware_interface::HW_IF_EFFORT && data.limited.effort.has_value())
            {
              return data.limited.effort.value();
            }
            else if (
              interface_name == hardware_interface::HW_IF_ACCELERATION &&
              data.limited.acceleration.has_value())
            {
              return data.limited.acceleration.value();
            }
            else
            {
              return value;
            }
          };
          interface->set_on_set_command_limiter(limiter_fn);
        }
      }
    }
  }

  /// Removes command interfaces from internal storage.
  /**
   * Command interface are removed from the maps with theirs storage and their claimed status.
   *
   * \param[interface_names] list of command interface names to remove from storage.
   */
  void remove_command_interfaces(const std::vector<std::string> & interface_names)
  {
    for (const auto & interface : interface_names)
    {
      command_interface_map_[interface]->unregisterIntrospection();
      command_interface_map_.erase(interface);
      claimed_command_interface_map_.erase(interface);
    }
  }

  // TODO(destogl): Propagate "false" up, if happens in initialize_hardware
  bool load_and_initialize_actuator(const HardwareInfo & hardware_info)
  {
    auto load_and_init_actuators = [&](auto & container)
    {
      if (!load_hardware<Actuator, ActuatorInterface>(hardware_info, actuator_loader_, container))
      {
        return false;
      }
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
        import_command_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "Actuator hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
        return false;
      }
      return true;
    };
    return load_and_init_actuators(actuators_);
  }

  bool load_and_initialize_sensor(const HardwareInfo & hardware_info)
  {
    auto load_and_init_sensors = [&](auto & container)
    {
      if (!load_hardware<Sensor, SensorInterface>(hardware_info, sensor_loader_, container))
      {
        return false;
      }
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "Sensor hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
        return false;
      }
      return true;
    };

    return load_and_init_sensors(sensors_);
  }

  bool load_and_initialize_system(const HardwareInfo & hardware_info)
  {
    auto load_and_init_systems = [&](auto & container)
    {
      if (!load_hardware<System, SystemInterface>(hardware_info, system_loader_, container))
      {
        return false;
      }
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
        import_command_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "System hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
        return false;
      }
      return true;
    };
    return load_and_init_systems(systems_);
  }

  void initialize_actuator(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
  {
    auto init_actuators = [&](auto & container)
    {
      container.emplace_back(Actuator(std::move(actuator)));
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
        import_command_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "Actuator hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
      }
    };

    init_actuators(actuators_);
  }

  void initialize_sensor(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
  {
    auto init_sensors = [&](auto & container)
    {
      container.emplace_back(Sensor(std::move(sensor)));
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "Sensor hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
      }
    };

    init_sensors(sensors_);
  }

  void initialize_system(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
  {
    auto init_systems = [&](auto & container)
    {
      container.emplace_back(System(std::move(system)));
      if (initialize_hardware(hardware_info, container.back()))
      {
        import_state_interfaces(container.back());
        import_command_interfaces(container.back());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "System hardware component '%s' from plugin '%s' failed to initialize.",
          hardware_info.name.c_str(), hardware_info.hardware_plugin_name.c_str());
      }
    };

    init_systems(systems_);
  }

  void clear()
  {
    actuators_.clear();
    sensors_.clear();
    systems_.clear();

    hardware_info_map_.clear();
    state_interface_map_.clear();
    command_interface_map_.clear();

    available_state_interfaces_.clear();
    available_command_interfaces_.clear();

    claimed_command_interface_map_.clear();
  }

  /**
   * Returns the return type of the hardware component group state, if the return type is other
   * than OK, then updates the return type of the group to the respective one
   */
  return_type update_hardware_component_group_state(
    const std::string & group_name, const return_type & value)
  {
    // This is for the components that has no configured group
    if (group_name.empty())
    {
      return value;
    }
    // If it is anything other than OK, change the return type of the hardware group state
    // to the respective return type
    if (value != return_type::OK)
    {
      hw_group_state_.at(group_name) = value;
    }
    return hw_group_state_.at(group_name);
  }

  /// Gets the logger for the resource storage
  /**
   * \return logger of the resource storage
   */
  const rclcpp::Logger & get_logger() const { return rm_logger_; }

  /// Gets the clock for the resource storage
  /**
   * \return clock of the resource storage
   */
  rclcpp::Clock::SharedPtr get_clock() const { return rm_clock_; }

  // hardware plugins
  pluginlib::ClassLoader<ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<SystemInterface> system_loader_;

  // Logger and Clock interfaces
  rclcpp::Clock::SharedPtr rm_clock_;
  rclcpp::Logger rm_logger_;

  std::vector<Actuator> actuators_;
  std::vector<Sensor> sensors_;
  std::vector<System> systems_;

  std::unordered_map<std::string, HardwareComponentInfo> hardware_info_map_;
  std::unordered_map<std::string, hardware_interface::return_type> hw_group_state_;

  /// Mapping between hardware and controllers that are using it (accessing data from it)
  std::unordered_map<std::string, std::vector<std::string>> hardware_used_by_controllers_;

  /// Mapping between controllers and list of interfaces they are using
  std::unordered_map<std::string, std::vector<std::string>>
    controllers_exported_state_interfaces_map_;
  std::unordered_map<std::string, std::vector<std::string>> controllers_reference_interfaces_map_;

  /// Storage of all available state interfaces
  std::map<std::string, StateInterface::ConstSharedPtr> state_interface_map_;
  /// Storage of all available command interfaces
  std::map<std::string, CommandInterface::SharedPtr> command_interface_map_;

  /// Vectors with interfaces available to controllers (depending on hardware component state)
  std::vector<std::string> available_state_interfaces_;
  std::vector<std::string> available_command_interfaces_;

  /// List of all claimed command interfaces
  std::unordered_map<std::string, bool> claimed_command_interface_map_;

  std::unordered_map<std::string, joint_limits::JointInterfacesCommandLimiterData> limiters_data_;

  std::unordered_map<
    std::string, std::unordered_map<
                   std::string, std::unique_ptr<joint_limits::JointLimiterInterface<
                                  joint_limits::JointControlInterfacesData>>>>
    joint_limiters_interface_;

  std::string robot_description_;

  /// The callback to be called when a component state is switched
  std::function<void()> on_component_state_switch_callback_ = nullptr;

  // To be used with the prepare and perform command switch for the hardware components
  std::vector<std::string> start_interfaces_buffer_;
  std::vector<std::string> stop_interfaces_buffer_;

  // Update rate of the controller manager, and the clock interface of its node
  // Used by async components.
  unsigned int cm_update_rate_ = 100;
};

ResourceManager::ResourceManager(
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface)
: resource_storage_(std::make_unique<ResourceStorage>(clock_interface, logger_interface))
{
}

ResourceManager::ResourceManager(rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
: resource_storage_(std::make_unique<ResourceStorage>(clock, logger))
{
}

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(
  const std::string & urdf, rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface, bool activate_all,
  const unsigned int update_rate)
: resource_storage_(std::make_unique<ResourceStorage>(clock_interface, logger_interface))
{
  resource_storage_->robot_description_ = urdf;
  load_and_initialize_components(urdf, update_rate);

  if (activate_all)
  {
    for (auto const & hw_info : resource_storage_->hardware_info_map_)
    {
      using lifecycle_msgs::msg::State;
      rclcpp_lifecycle::State state(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
      set_component_state(hw_info.first, state);
    }
  }
}

ResourceManager::ResourceManager(
  const std::string & urdf, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger,
  bool activate_all, const unsigned int update_rate)
: resource_storage_(std::make_unique<ResourceStorage>(clock, logger))
{
  load_and_initialize_components(urdf, update_rate);

  if (activate_all)
  {
    for (auto const & hw_info : resource_storage_->hardware_info_map_)
    {
      using lifecycle_msgs::msg::State;
      rclcpp_lifecycle::State state(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
      set_component_state(hw_info.first, state);
    }
  }
}

bool ResourceManager::shutdown_components()
{
  std::unique_lock<std::recursive_mutex> guard(resource_interfaces_lock_);
  bool shutdown_status = true;
  for (auto const & hw_info : resource_storage_->hardware_info_map_)
  {
    rclcpp_lifecycle::State finalized_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);
    if (set_component_state(hw_info.first, finalized_state) != return_type::OK)
    {
      shutdown_status = false;
    }
  }
  return shutdown_status;
}

// CM API: Called in "callback/slow"-thread
bool ResourceManager::load_and_initialize_components(
  const std::string & urdf, const unsigned int update_rate)
{
  components_are_loaded_and_initialized_ = true;

  resource_storage_->robot_description_ = urdf;
  resource_storage_->cm_update_rate_ = update_rate;

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  // Set the update rate for all hardware components
  for (auto & hw : hardware_info)
  {
    hw.rw_rate = (hw.rw_rate == 0 || hw.rw_rate > update_rate) ? update_rate : hw.rw_rate;
  }

  const std::string system_type = "system";
  const std::string sensor_type = "sensor";
  const std::string actuator_type = "actuator";

  std::lock_guard<std::recursive_mutex> resource_guard(resources_lock_);
  for (const auto & individual_hardware_info : hardware_info)
  {
    // Check for identical names
    if (
      resource_storage_->hardware_info_map_.find(individual_hardware_info.name) !=
      resource_storage_->hardware_info_map_.end())
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager",
        "Hardware name %s is duplicated. Please provide a unique 'name' "
        "in the URDF.",
        individual_hardware_info.name.c_str());
      components_are_loaded_and_initialized_ = false;
      break;
    }

    if (individual_hardware_info.type == actuator_type)
    {
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
      if (!resource_storage_->load_and_initialize_actuator(individual_hardware_info))
      {
        components_are_loaded_and_initialized_ = false;
        break;
      }
    }
    if (individual_hardware_info.type == sensor_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      if (!resource_storage_->load_and_initialize_sensor(individual_hardware_info))
      {
        components_are_loaded_and_initialized_ = false;
        break;
      }
    }
    if (individual_hardware_info.type == system_type)
    {
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
      if (!resource_storage_->load_and_initialize_system(individual_hardware_info))
      {
        components_are_loaded_and_initialized_ = false;
        break;
      }
    }
  }

  if (components_are_loaded_and_initialized_ && validate_storage(hardware_info))
  {
    std::lock_guard<std::recursive_mutex> guard(resources_lock_);
    read_write_status.failed_hardware_names.reserve(
      resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
      resource_storage_->systems_.size());
  }
  else
  {
    std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
    resource_storage_->clear();
    // cleanup everything
    components_are_loaded_and_initialized_ = false;
  }

  return components_are_loaded_and_initialized_;
}

void ResourceManager::import_joint_limiters(const std::string & urdf)
{
  std::lock_guard<std::recursive_mutex> guard(joint_limiters_lock_);
  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  resource_storage_->import_joint_limiters(hardware_info);
}

bool ResourceManager::are_components_initialized() const
{
  return components_are_loaded_and_initialized_;
}

// CM API: Called in "update"-thread
LoanedStateInterface ResourceManager::claim_state_interface(const std::string & key)
{
  if (!state_interface_is_available(key))
  {
    throw std::runtime_error(
      fmt::format(FMT_COMPILE("State interface with key '{}' does not exist"), key));
  }

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedStateInterface(resource_storage_->state_interface_map_.at(key));
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::state_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->state_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::available_state_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_state_interfaces_;
}

// CM API: Called in "update"-thread (indirectly through `claim_state_interface`)
bool ResourceManager::state_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_state_interfaces_.begin(),
           resource_storage_->available_state_interfaces_.end(),
           name) != resource_storage_->available_state_interfaces_.end();
}

std::string ResourceManager::get_state_interface_data_type(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  auto it = resource_storage_->state_interface_map_.find(name);
  if (it != resource_storage_->state_interface_map_.end())
  {
    return it->second->get_data_type().to_string();
  }
  else
  {
    throw std::runtime_error(
      std::string("State interface with key '") + name + std::string("' does not exist"));
  }
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::import_controller_exported_state_interfaces(
  const std::string & controller_name, std::vector<StateInterface::ConstSharedPtr> & interfaces)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  auto interface_names = resource_storage_->add_state_interfaces(interfaces);
  resource_storage_->controllers_exported_state_interfaces_map_[controller_name] = interface_names;
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::get_controller_exported_state_interface_names(
  const std::string & controller_name)
{
  return resource_storage_->controllers_exported_state_interfaces_map_.at(controller_name);
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_exported_state_interfaces_available(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_exported_state_interfaces_map_.at(controller_name);
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->available_state_interfaces_.insert(
    resource_storage_->available_state_interfaces_.end(), interface_names.begin(),
    interface_names.end());
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_exported_state_interfaces_unavailable(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_exported_state_interfaces_map_.at(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & interface : interface_names)
  {
    auto found_it = std::find(
      resource_storage_->available_state_interfaces_.begin(),
      resource_storage_->available_state_interfaces_.end(), interface);
    if (found_it != resource_storage_->available_state_interfaces_.end())
    {
      resource_storage_->available_state_interfaces_.erase(found_it);
      RCUTILS_LOG_DEBUG_NAMED(
        "resource_manager", "'%s' state interface removed from available list", interface.c_str());
    }
  }
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::remove_controller_exported_state_interfaces(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_exported_state_interfaces_map_.at(controller_name);
  resource_storage_->controllers_exported_state_interfaces_map_.erase(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->remove_state_interfaces(interface_names);
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::import_controller_reference_interfaces(
  const std::string & controller_name,
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & interfaces)
{
  std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
  auto interface_names = resource_storage_->add_command_interfaces(interfaces);
  resource_storage_->controllers_reference_interfaces_map_[controller_name] = interface_names;
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::get_controller_reference_interface_names(
  const std::string & controller_name)
{
  return resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_reference_interfaces_available(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->available_command_interfaces_.insert(
    resource_storage_->available_command_interfaces_.end(), interface_names.begin(),
    interface_names.end());
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_reference_interfaces_unavailable(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & interface : interface_names)
  {
    auto found_it = std::find(
      resource_storage_->available_command_interfaces_.begin(),
      resource_storage_->available_command_interfaces_.end(), interface);
    if (found_it != resource_storage_->available_command_interfaces_.end())
    {
      resource_storage_->available_command_interfaces_.erase(found_it);
      RCLCPP_DEBUG(
        get_logger(), "'%s' command interface removed from available list", interface.c_str());
    }
  }
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::remove_controller_reference_interfaces(const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  resource_storage_->controllers_reference_interfaces_map_.erase(controller_name);

  std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
  resource_storage_->remove_command_interfaces(interface_names);
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::cache_controller_to_hardware(
  const std::string & controller_name, const std::vector<std::string> & interfaces)
{
  for (const auto & interface : interfaces)
  {
    bool found = false;
    for (const auto & [hw_name, hw_info] : resource_storage_->hardware_info_map_)
    {
      auto cmd_itf_it =
        std::find(hw_info.command_interfaces.begin(), hw_info.command_interfaces.end(), interface);
      if (cmd_itf_it != hw_info.command_interfaces.end())
      {
        found = true;
      }
      auto state_itf_it =
        std::find(hw_info.state_interfaces.begin(), hw_info.state_interfaces.end(), interface);
      if (state_itf_it != hw_info.state_interfaces.end())
      {
        found = true;
      }

      if (found)
      {
        // check if controller exist already in the list and if not add it
        auto controllers = resource_storage_->hardware_used_by_controllers_[hw_name];
        auto ctrl_it = std::find(controllers.begin(), controllers.end(), controller_name);
        if (ctrl_it == controllers.end())
        {
          // add because it does not exist
          controllers.reserve(controllers.size() + 1);
          controllers.push_back(controller_name);
        }
        resource_storage_->hardware_used_by_controllers_[hw_name] = controllers;
        break;
      }
    }
  }
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::get_cached_controllers_to_hardware(
  const std::string & hardware_name)
{
  return resource_storage_->hardware_used_by_controllers_[hardware_name];
}

// CM API: Called in "update"-thread
bool ResourceManager::command_interface_is_claimed(const std::string & key) const
{
  if (!command_interface_is_available(key))
  {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  return resource_storage_->claimed_command_interface_map_.at(key);
}

// CM API: Called in "update"-thread
LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
{
  if (!command_interface_is_available(key))
  {
    throw std::runtime_error(
      fmt::format(FMT_COMPILE("Command interface with key '{}' does not exist"), key));
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  if (command_interface_is_claimed(key))
  {
    throw std::runtime_error(
      fmt::format(FMT_COMPILE("Command interface with key '{}' is already claimed"), key));
  }

  resource_storage_->claimed_command_interface_map_[key] = true;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedCommandInterface(
    resource_storage_->command_interface_map_.at(key),
    std::bind(&ResourceManager::release_command_interface, this, key));
}

// CM API: Called in "update"-thread
void ResourceManager::release_command_interface(const std::string & key)
{
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  resource_storage_->claimed_command_interface_map_[key] = false;
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::command_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->command_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::available_command_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_command_interfaces_;
}

// CM API: Called in "callback/slow"-thread
bool ResourceManager::command_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_command_interfaces_.begin(),
           resource_storage_->available_command_interfaces_.end(),
           name) != resource_storage_->available_command_interfaces_.end();
}

std::string ResourceManager::get_command_interface_data_type(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  auto it = resource_storage_->command_interface_map_.find(name);
  if (it != resource_storage_->command_interface_map_.end())
  {
    return it->second->get_data_type().to_string();
  }
  else
  {
    throw std::runtime_error(
      std::string("Command interface with '") + name + std::string("' does not exist"));
  }
}

void ResourceManager::import_component(
  std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_actuator(std::move(actuator), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

void ResourceManager::import_component(
  std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_sensor(std::move(sensor), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

void ResourceManager::import_component(
  std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_system(std::move(system), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

// CM API: Called in "callback/slow"-thread
const std::unordered_map<std::string, HardwareComponentInfo> &
ResourceManager::get_components_status()
{
  auto loop_and_get_state = [&](auto & container)
  {
    for (auto & component : container)
    {
      resource_storage_->hardware_info_map_[component.get_name()].state =
        component.get_lifecycle_state();
    }
  };

  loop_and_get_state(resource_storage_->actuators_);
  loop_and_get_state(resource_storage_->sensors_);
  loop_and_get_state(resource_storage_->systems_);

  return resource_storage_->hardware_info_map_;
}

// CM API: Called in "callback/slow"-thread
bool ResourceManager::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // When only broadcaster is activated then this lists are empty
  if (start_interfaces.empty() && stop_interfaces.empty())
  {
    return true;
  }

  // Check if interface exists
  std::stringstream ss_not_existing;
  ss_not_existing << "Not existing: " << std::endl << "[" << std::endl;
  auto check_exist = [&](const std::vector<std::string> & list_to_check)
  {
    bool all_exist = true;
    for (const auto & interface : list_to_check)
    {
      if (!command_interface_exists(interface))
      {
        all_exist = false;
        ss_not_existing << " " << interface << std::endl;
      }
    }
    return all_exist;
  };
  if (!(check_exist(start_interfaces) && check_exist(stop_interfaces)))
  {
    ss_not_existing << "]" << std::endl;
    RCLCPP_ERROR(
      get_logger(), "Not acceptable command interfaces combination: \n%s%s",
      interfaces_to_string(start_interfaces, stop_interfaces).c_str(),
      ss_not_existing.str().c_str());
    return false;
  }

  // Check if interfaces are available
  std::stringstream ss_not_available;
  ss_not_available << "Not available: " << std::endl << "[" << std::endl;
  auto check_available = [&](const std::vector<std::string> & list_to_check)
  {
    bool all_available = true;
    for (const auto & interface : list_to_check)
    {
      if (!command_interface_is_available(interface))
      {
        all_available = false;
        ss_not_available << " " << interface << std::endl;
      }
    }
    return all_available;
  };
  if (!(check_available(start_interfaces) && check_available(stop_interfaces)))
  {
    ss_not_available << "]" << std::endl;
    RCLCPP_ERROR(
      get_logger(), "Not acceptable command interfaces combination: \n%s%s",
      interfaces_to_string(start_interfaces, stop_interfaces).c_str(),
      ss_not_available.str().c_str());
    return false;
  }

  const auto & hardware_info_map = resource_storage_->hardware_info_map_;
  auto call_prepare_mode_switch =
    [&start_interfaces, &stop_interfaces, &hardware_info_map, logger = get_logger()](
      auto & components, auto & start_interfaces_buffer, auto & stop_interfaces_buffer)
  {
    bool ret = true;
    for (auto & component : components)
    {
      const auto & hw_command_itfs = hardware_info_map.at(component.get_name()).command_interfaces;
      find_common_hardware_interfaces(hw_command_itfs, start_interfaces, start_interfaces_buffer);
      find_common_hardware_interfaces(hw_command_itfs, stop_interfaces, stop_interfaces_buffer);
      if (start_interfaces_buffer.empty() && stop_interfaces_buffer.empty())
      {
        RCLCPP_DEBUG(
          logger, "Component '%s' after filtering has no command interfaces to switch",
          component.get_name().c_str());
        continue;
      }
      if (
        component.get_lifecycle_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
        component.get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        try
        {
          if (
            return_type::OK !=
            component.prepare_command_mode_switch(start_interfaces_buffer, stop_interfaces_buffer))
          {
            RCLCPP_ERROR(
              logger, "Component '%s' did not accept command interfaces combination: \n%s",
              component.get_name().c_str(),
              interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str());
            ret = false;
          }
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(
            logger,
            "Exception of type : %s occurred while preparing command mode switch for component "
            "'%s' for the interfaces: \n %s : %s",
            typeid(e).name(), component.get_name().c_str(),
            interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str(),
            e.what());
          ret = false;
        }
        catch (...)
        {
          RCLCPP_ERROR(
            logger,
            "Unknown exception occurred while preparing command mode switch for component '%s' for "
            "the interfaces: \n %s",
            component.get_name().c_str(),
            interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str());
          ret = false;
        }
      }
      else
      {
        RCLCPP_WARN(
          logger, "Component '%s' is not in INACTIVE or ACTIVE state, skipping the prepare switch",
          component.get_name().c_str());
        ret = false;
      }
    }
    return ret;
  };

  const bool actuators_result = call_prepare_mode_switch(
    resource_storage_->actuators_, resource_storage_->start_interfaces_buffer_,
    resource_storage_->stop_interfaces_buffer_);
  const bool systems_result = call_prepare_mode_switch(
    resource_storage_->systems_, resource_storage_->start_interfaces_buffer_,
    resource_storage_->stop_interfaces_buffer_);

  return actuators_result && systems_result;
}

// CM API: Called in "update"-thread
bool ResourceManager::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // When only broadcaster is activated then this lists are empty
  if (start_interfaces.empty() && stop_interfaces.empty())
  {
    return true;
  }

  const auto & hardware_info_map = resource_storage_->hardware_info_map_;
  auto call_perform_mode_switch =
    [&start_interfaces, &stop_interfaces, &hardware_info_map, logger = get_logger()](
      auto & components, auto & start_interfaces_buffer, auto & stop_interfaces_buffer)
  {
    bool ret = true;
    for (auto & component : components)
    {
      const auto & hw_command_itfs = hardware_info_map.at(component.get_name()).command_interfaces;
      find_common_hardware_interfaces(hw_command_itfs, start_interfaces, start_interfaces_buffer);
      find_common_hardware_interfaces(hw_command_itfs, stop_interfaces, stop_interfaces_buffer);
      if (start_interfaces_buffer.empty() && stop_interfaces_buffer.empty())
      {
        RCLCPP_DEBUG(
          logger, "Component '%s' after filtering has no command interfaces to perform switch",
          component.get_name().c_str());
        continue;
      }
      if (
        component.get_lifecycle_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
        component.get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        try
        {
          if (
            return_type::OK !=
            component.perform_command_mode_switch(start_interfaces_buffer, stop_interfaces_buffer))
          {
            RCLCPP_ERROR(
              logger, "Component '%s' could not perform switch for the command interfaces: \n%s",
              component.get_name().c_str(),
              interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str());
            ret = false;
          }
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(
            logger,
            "Exception of type : %s occurred while performing command mode switch for component "
            "'%s' for the interfaces: \n %s : %s",
            typeid(e).name(), component.get_name().c_str(),
            interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str(),
            e.what());
          ret = false;
        }
        catch (...)
        {
          RCLCPP_ERROR(
            logger,
            "Unknown exception occurred while performing command mode switch for component '%s' "
            "for "
            "the interfaces: \n %s",
            component.get_name().c_str(),
            interfaces_to_string(start_interfaces_buffer, stop_interfaces_buffer).c_str());
          ret = false;
        }
      }
      else
      {
        RCLCPP_WARN(
          logger, "Component '%s' is not in INACTIVE or ACTIVE state, skipping the perform switch",
          component.get_name().c_str());
        ret = false;
      }
    }
    return ret;
  };

  const bool actuators_result = call_perform_mode_switch(
    resource_storage_->actuators_, resource_storage_->start_interfaces_buffer_,
    resource_storage_->stop_interfaces_buffer_);
  const bool systems_result = call_perform_mode_switch(
    resource_storage_->systems_, resource_storage_->start_interfaces_buffer_,
    resource_storage_->stop_interfaces_buffer_);

  if (actuators_result && systems_result)
  {
    // Reset the internals of the joint limiters
    for (auto & [hw_name, limiters] : resource_storage_->joint_limiters_interface_)
    {
      for (const auto & [joint_name, limiter] : limiters)
      {
        limiter->reset_internals();
        RCLCPP_DEBUG(
          get_logger(), "Resetting internals of joint limiter for joint '%s' in hardware '%s'",
          joint_name.c_str(), hw_name.c_str());
      }
    }
  }

  return actuators_result && systems_result;
}

// CM API: Called in "callback/slow"-thread
return_type ResourceManager::set_component_state(
  const std::string & component_name, rclcpp_lifecycle::State & target_state)
{
  using lifecycle_msgs::msg::State;
  using std::placeholders::_1;
  using std::placeholders::_2;

  auto found_it = resource_storage_->hardware_info_map_.find(component_name);

  if (found_it == resource_storage_->hardware_info_map_.end())
  {
    RCLCPP_INFO(
      get_logger(), "Hardware Component with name '%s' does not exists", component_name.c_str());
    return return_type::ERROR;
  }

  return_type result = return_type::OK;

  if (target_state.id() == 0)
  {
    if (target_state.label() == lifecycle_state_names::UNCONFIGURED)
    {
      target_state = rclcpp_lifecycle::State(
        State::PRIMARY_STATE_UNCONFIGURED, lifecycle_state_names::UNCONFIGURED);
    }
    if (target_state.label() == lifecycle_state_names::INACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::ACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::FINALIZED)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);
    }
  }

  auto find_set_component_state = [&](auto action, auto & components)
  {
    auto found_component_it = std::find_if(
      components.begin(), components.end(),
      [&](const auto & component) { return component.get_name() == component_name; });

    if (found_component_it != components.end())
    {
      if (action(*found_component_it, target_state))
      {
        result = return_type::OK;
      }
      else
      {
        result = return_type::ERROR;
      }
      return true;
    }
    return false;
  };

  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  bool found = find_set_component_state(
    std::bind(&ResourceStorage::set_component_state<Actuator>, resource_storage_.get(), _1, _2),
    resource_storage_->actuators_);
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<Sensor>, resource_storage_.get(), _1, _2),
      resource_storage_->sensors_);
  }
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<System>, resource_storage_.get(), _1, _2),
      resource_storage_->systems_);
  }

  return result;
}

// CM API: Called in "update"-thread
bool ResourceManager::enforce_command_limits(const rclcpp::Duration & period)
{
  std::unique_lock<std::recursive_mutex> limiters_guard(joint_limiters_lock_, std::try_to_lock);
  if (!limiters_guard.owns_lock())
  {
    return false;
  }

  bool enforce_result = false;
  // Joint Limiters operations
  for (auto & [hw_name, limiters] : resource_storage_->joint_limiters_interface_)
  {
    for (const auto & [joint_name, limiter] : limiters)
    {
      enforce_result |= resource_storage_->enforce_command_limits(joint_name, period);
    }
  }
  return enforce_result;
}

// CM API: Called in "update"-thread
HardwareReadWriteStatus ResourceManager::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  read_write_status.ok = true;
  read_write_status.failed_hardware_names.clear();

  // This is needed while we load and initialize the components
  std::unique_lock<std::recursive_mutex> resource_guard(resources_lock_, std::try_to_lock);
  if (!resource_guard.owns_lock())
  {
    return read_write_status;
  }
  auto read_components = [&](auto & components)
  {
    for (auto & component : components)
    {
      std::unique_lock<std::recursive_mutex> lock(component.get_mutex(), std::try_to_lock);
      const std::string component_name = component.get_name();
      if (!lock.owns_lock())
      {
        RCLCPP_DEBUG(
          get_logger(), "Skipping read() call for the component '%s' since it is locked",
          component_name.c_str());
        continue;
      }
      auto ret_val = return_type::OK;
      try
      {
        auto & hardware_component_info = resource_storage_->hardware_info_map_[component_name];
        const auto current_time = resource_storage_->get_clock()->now();
        if (
          hardware_component_info.rw_rate == 0 ||
          hardware_component_info.rw_rate == resource_storage_->cm_update_rate_)
        {
          ret_val = component.read(current_time, period);
        }
        else
        {
          const double read_rate = hardware_component_info.rw_rate;
          const rclcpp::Duration actual_period =
            component.get_last_read_time().get_clock_type() != RCL_CLOCK_UNINITIALIZED
              ? current_time - component.get_last_read_time()
              : rclcpp::Duration::from_seconds(1.0 / static_cast<double>(read_rate));

          const double error_now = std::abs(actual_period.seconds() * read_rate - 1.0);
          const double error_if_skipped = std::abs(
            (actual_period.seconds() + 1.0 / resource_storage_->cm_update_rate_) * read_rate - 1.0);
          if (error_now <= error_if_skipped)
          {
            ret_val = component.read(current_time, actual_period);
          }
        }
        if (hardware_component_info.read_statistics)
        {
          const auto & read_statistics_collector = component.get_read_statistics();
          hardware_component_info.read_statistics->execution_time.update_statistics(
            read_statistics_collector.execution_time);
          hardware_component_info.read_statistics->periodicity.update_statistics(
            read_statistics_collector.periodicity);
        }
        const auto component_group = component.get_group_name();
        ret_val =
          resource_storage_->update_hardware_component_group_state(component_group, ret_val);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          get_logger(), "Exception of type : %s thrown during read of the component '%s': %s",
          typeid(e).name(), component_name.c_str(), e.what());
        ret_val = return_type::ERROR;
      }
      catch (...)
      {
        RCLCPP_ERROR(
          get_logger(), "Unknown exception thrown during read of the component '%s'",
          component_name.c_str());
        ret_val = return_type::ERROR;
      }
      if (ret_val == return_type::ERROR)
      {
        component.error();
        read_write_status.ok = false;
        read_write_status.failed_hardware_names.push_back(component_name);
        resource_storage_->remove_all_hardware_interfaces_from_available_list(component_name);
      }
      else if (ret_val == return_type::DEACTIVATE)
      {
        rclcpp_lifecycle::State inactive_state(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
        set_component_state(component_name, inactive_state);
      }
      // If desired: automatic re-activation. We could add a flag for this...
      // else
      // {
      // using lifecycle_msgs::msg::State;
      // rclcpp_lifecycle::State state(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
      // set_component_state(component.get_name(), state);
      // }
    }
  };

  read_components(resource_storage_->actuators_);
  read_components(resource_storage_->sensors_);
  read_components(resource_storage_->systems_);

  return read_write_status;
}

// CM API: Called in "update"-thread
HardwareReadWriteStatus ResourceManager::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  read_write_status.ok = true;
  read_write_status.failed_hardware_names.clear();

  // This is needed while we load and initialize the components
  std::unique_lock<std::recursive_mutex> resource_guard(resources_lock_, std::try_to_lock);
  if (!resource_guard.owns_lock())
  {
    return read_write_status;
  }
  auto write_components = [&](auto & components)
  {
    for (auto & component : components)
    {
      std::unique_lock<std::recursive_mutex> lock(component.get_mutex(), std::try_to_lock);
      const std::string component_name = component.get_name();
      if (!lock.owns_lock())
      {
        RCLCPP_DEBUG(
          get_logger(), "Skipping write() call for the component '%s' since it is locked",
          component_name.c_str());
        continue;
      }
      auto ret_val = return_type::OK;
      try
      {
        auto & hardware_component_info = resource_storage_->hardware_info_map_[component_name];
        const auto current_time = resource_storage_->get_clock()->now();
        if (
          hardware_component_info.rw_rate == 0 ||
          hardware_component_info.rw_rate == resource_storage_->cm_update_rate_)
        {
          ret_val = component.write(current_time, period);
        }
        else
        {
          const double write_rate = hardware_component_info.rw_rate;
          const rclcpp::Duration actual_period =
            component.get_last_write_time().get_clock_type() != RCL_CLOCK_UNINITIALIZED
              ? current_time - component.get_last_write_time()
              : rclcpp::Duration::from_seconds(1.0 / static_cast<double>(write_rate));

          const double error_now = std::abs(actual_period.seconds() * write_rate - 1.0);
          const double error_if_skipped = std::abs(
            (actual_period.seconds() + 1.0 / resource_storage_->cm_update_rate_) * write_rate -
            1.0);
          if (error_now <= error_if_skipped)
          {
            ret_val = component.write(current_time, actual_period);
          }
        }
        if (hardware_component_info.write_statistics)
        {
          const auto & write_statistics_collector = component.get_write_statistics();
          hardware_component_info.write_statistics->execution_time.update_statistics(
            write_statistics_collector.execution_time);
          hardware_component_info.write_statistics->periodicity.update_statistics(
            write_statistics_collector.periodicity);
        }
        const auto component_group = component.get_group_name();
        ret_val =
          resource_storage_->update_hardware_component_group_state(component_group, ret_val);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          get_logger(), "Exception of type : %s thrown during write of the component '%s': %s",
          typeid(e).name(), component_name.c_str(), e.what());
        ret_val = return_type::ERROR;
      }
      catch (...)
      {
        RCLCPP_ERROR(
          get_logger(), "Unknown exception thrown during write of the component '%s'",
          component_name.c_str());
        ret_val = return_type::ERROR;
      }
      if (ret_val == return_type::ERROR)
      {
        component.error();
        read_write_status.ok = false;
        read_write_status.failed_hardware_names.push_back(component_name);
        resource_storage_->remove_all_hardware_interfaces_from_available_list(component_name);
      }
      else if (ret_val == return_type::DEACTIVATE)
      {
        rclcpp_lifecycle::State inactive_state(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
        set_component_state(component_name, inactive_state);
      }
    }
  };

  write_components(resource_storage_->actuators_);
  write_components(resource_storage_->systems_);

  return read_write_status;
}

// BEGIN: "used only in tests and locally"
size_t ResourceManager::actuator_components_size() const
{
  return resource_storage_->actuators_.size();
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensors_.size();
}

size_t ResourceManager::system_components_size() const
{
  return resource_storage_->systems_.size();
}

bool ResourceManager::command_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->command_interface_map_.find(key) !=
         resource_storage_->command_interface_map_.end();
}

bool ResourceManager::state_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->state_interface_map_.find(key) !=
         resource_storage_->state_interface_map_.end();
}

void ResourceManager::set_on_component_state_switch_callback(std::function<void()> callback)
{
  resource_storage_->on_component_state_switch_callback_ = callback;
}

const std::string & ResourceManager::get_robot_description() const
{
  return resource_storage_->robot_description_;
}

// END: "used only in tests and locally"

rclcpp::Logger ResourceManager::get_logger() const { return resource_storage_->get_logger(); }

rclcpp::Clock::SharedPtr ResourceManager::get_clock() const
{
  return resource_storage_->get_clock();
}

// BEGIN: private methods

bool ResourceManager::validate_storage(
  const std::vector<hardware_interface::HardwareInfo> & hardware_info) const
{
  std::vector<std::string> missing_state_keys = {};
  std::vector<std::string> missing_command_keys = {};

  for (const auto & hardware : hardware_info)
  {
    for (const auto & joint : hardware.joints)
    {
      for (const auto & state_interface : joint.state_interfaces)
      {
        if (!state_interface_exists(joint.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(joint.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : joint.command_interfaces)
      {
        if (!command_interface_exists(joint.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(joint.name + "/" + command_interface.name);
        }
      }
    }
    for (const auto & sensor : hardware.sensors)
    {
      for (const auto & state_interface : sensor.state_interfaces)
      {
        if (!state_interface_exists(sensor.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(sensor.name + "/" + state_interface.name);
        }
      }
    }
    for (const auto & gpio : hardware.gpios)
    {
      for (const auto & state_interface : gpio.state_interfaces)
      {
        if (!state_interface_exists(gpio.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(gpio.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : gpio.command_interfaces)
      {
        if (!command_interface_exists(gpio.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(gpio.name + "/" + command_interface.name);
        }
      }
    }
  }

  if (!missing_state_keys.empty() || !missing_command_keys.empty())
  {
    std::string message = "Wrong state or command interface configuration.\n";
    message += "missing state interfaces:\n";
    for (const auto & missing_key : missing_state_keys)
    {
      message += " " + missing_key + "\t";
    }
    message += "\nmissing command interfaces:\n";
    for (const auto & missing_key : missing_command_keys)
    {
      message += " " + missing_key + "\t";
    }

    RCUTILS_LOG_ERROR_NAMED(
      "resource_manager",
      "Discrepancy between robot description file (urdf) and actually exported HW interfaces.\n"
      "Details: %s",
      message.c_str());

    return false;
  }

  return true;
}

// END: private methods

}  // namespace hardware_interface
