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

/// \file benchmark_handle_access.cpp
/// \brief Benchmark suite for hardware interface access patterns (ros2_control #2816)
///
/// This benchmark characterizes and compares PR #2831 cached-handle path vs string-based path,
/// measuring the performance of state/command interface access in hardware components.
///
/// For stable results, pin to a single core and set performance governor:
///   taskset -c 0 ./benchmark_handle_access --benchmark_repetitions=5
///   echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
///
/// All Tier 2 benchmarks use wait_until_*=true (the safe production default and what #2816
/// reporter measured). Users can add wait=false variants for comparison if needed.

#include <algorithm>
#include <atomic>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "benchmark/benchmark.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

// Process-wide failure flag — ensures CTest/CI fails on any setup error
static std::atomic<bool> g_setup_failed{false};

// ============================================================================
// Benchmark Scenarios
// ============================================================================

struct Scenario
{
  const char * name;
  size_t num_state;
  size_t num_command;
};

// Interface count scenarios matching the plan
constexpr Scenario kScenarios[] = {
  {"small_arm", 12, 8},     // 4-DOF manipulator
  {"7dof_arm", 21, 14},     // KUKA LBR
  {"issue_2816", 67, 20},   // Reporter's exact setup
  {"humanoid_30", 90, 60},  // Legs + torso + arms
  {"humanoid_50", 150, 100} // Full humanoid + hands
};

constexpr size_t kNumScenarios = sizeof(kScenarios) / sizeof(kScenarios[0]);

// ============================================================================
// BenchmarkSystem — minimal SystemInterface for benchmarking
// ============================================================================

class BenchmarkSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }
};

// ============================================================================
// Helper: Build HardwareInfo programmatically
// ============================================================================

hardware_interface::HardwareInfo build_hardware_info(
  size_t num_state_interfaces, size_t num_command_interfaces)
{
  hardware_interface::HardwareInfo info;
  info.name = "benchmark_system";
  info.type = "system";
  info.rw_rate = 1000;
  info.is_async = false;

  // For (67, 20) exact case and others, we create interfaces directly.
  // We'll use a mix of joints with varying interface counts to achieve the target totals.
  // Simpler approach: create one "mega joint" with all interfaces.
  // For more realistic names, we'll create multiple joints.

  // Strategy: create joints until we have enough state/command interfaces
  // Each joint gets position/velocity/effort states and position/velocity commands
  // Adjust last joint to hit exact counts

  const std::vector<std::string> state_types = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };
  const std::vector<std::string> command_types = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY
  };

  size_t states_created = 0;
  size_t commands_created = 0;
  size_t joint_idx = 0;

  while (states_created < num_state_interfaces || commands_created < num_command_interfaces)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(joint_idx);
    joint.type = "joint";

    // Add state interfaces
    for (const auto & st : state_types)
    {
      if (states_created < num_state_interfaces)
      {
        hardware_interface::InterfaceInfo si;
        si.name = st;
        si.data_type = "double";
        joint.state_interfaces.push_back(si);
        ++states_created;
      }
    }

    // Add command interfaces
    for (const auto & ct : command_types)
    {
      if (commands_created < num_command_interfaces)
      {
        hardware_interface::InterfaceInfo ci;
        ci.name = ct;
        ci.data_type = "double";
        joint.command_interfaces.push_back(ci);
        ++commands_created;
      }
    }

    if (!joint.state_interfaces.empty() || !joint.command_interfaces.empty())
    {
      info.joints.push_back(joint);
    }
    ++joint_idx;
  }

  return info;
}

// ============================================================================
// Tier 1: Primitive Microbenchmarks Fixture
// ============================================================================

class PrimitiveBenchmarkFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    setup_error_.clear();

    const size_t scenario_idx = static_cast<size_t>(state.range(0));
    if (scenario_idx >= kNumScenarios)
    {
      setup_error_ = "Invalid scenario index";
      g_setup_failed.store(true);
      return;
    }

    const auto & scenario = kScenarios[scenario_idx];
    num_state_ = scenario.num_state;
    num_command_ = scenario.num_command;

    // Create raw double storage
    raw_state_values_.resize(num_state_, 0.0);
    raw_command_values_.resize(num_command_, 0.0);

    // Create raw pointers
    raw_state_ptrs_.clear();
    raw_command_ptrs_.clear();
    for (size_t i = 0; i < num_state_; ++i)
    {
      raw_state_ptrs_.push_back(&raw_state_values_[i]);
    }
    for (size_t i = 0; i < num_command_; ++i)
    {
      raw_command_ptrs_.push_back(&raw_command_values_[i]);
    }

    // Create handles with InterfaceDescription
    state_handles_.clear();
    command_handles_.clear();
    state_handle_map_.clear();
    command_handle_map_.clear();

    for (size_t i = 0; i < num_state_; ++i)
    {
      const std::string prefix = "joint" + std::to_string(i / 3);
      const std::string iface = (i % 3 == 0) ? "position" : ((i % 3 == 1) ? "velocity" : "effort");
      hardware_interface::InterfaceInfo ii;
      ii.name = iface;
      ii.data_type = "double";
      hardware_interface::InterfaceDescription desc(prefix, ii);

      auto handle = std::make_shared<hardware_interface::StateInterface>(desc);
      state_handles_.push_back(handle);
      state_handle_map_[desc.get_name()] = handle;
    }

    for (size_t i = 0; i < num_command_; ++i)
    {
      const std::string prefix = "joint" + std::to_string(i / 2);
      const std::string iface = (i % 2 == 0) ? "position" : "velocity";
      hardware_interface::InterfaceInfo ii;
      ii.name = iface;
      ii.data_type = "double";
      hardware_interface::InterfaceDescription desc(prefix, ii);

      auto handle = std::make_shared<hardware_interface::CommandInterface>(desc);
      command_handles_.push_back(handle);
      command_handle_map_[desc.get_name()] = handle;
    }
  }

  void TearDown(benchmark::State & /*state*/) override
  {
    raw_state_values_.clear();
    raw_command_values_.clear();
    raw_state_ptrs_.clear();
    raw_command_ptrs_.clear();
    state_handles_.clear();
    command_handles_.clear();
    state_handle_map_.clear();
    command_handle_map_.clear();
  }

protected:
  std::string setup_error_;
  size_t num_state_ = 0;
  size_t num_command_ = 0;

  // Raw pointer storage
  std::vector<double> raw_state_values_;
  std::vector<double> raw_command_values_;
  std::vector<double *> raw_state_ptrs_;
  std::vector<double *> raw_command_ptrs_;

  // Handle storage
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::unordered_map<std::string, hardware_interface::StateInterface::SharedPtr> state_handle_map_;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr>
    command_handle_map_;
};

// ============================================================================
// Tier 1 Benchmarks: Primitive Cost Decomposition
// ============================================================================

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_ReadRawPointer)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double sum = 0.0;
  for (auto _ : state)
  {
    for (size_t i = 0; i < num_state_; ++i)
    {
      sum += *raw_state_ptrs_[i];
    }
    for (size_t i = 0; i < num_command_; ++i)
    {
      sum += *raw_command_ptrs_[i];
    }
    benchmark::DoNotOptimize(sum);
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_WriteRawPointer)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double val = 42.0;
  for (auto _ : state)
  {
    for (size_t i = 0; i < num_state_; ++i)
    {
      *raw_state_ptrs_[i] = val;
    }
    for (size_t i = 0; i < num_command_; ++i)
    {
      *raw_command_ptrs_[i] = val;
    }
    benchmark::ClobberMemory();
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_ReadHandleDirect)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double val = 0.0;
  for (auto _ : state)
  {
    for (const auto & h : state_handles_)
    {
      std::ignore = h->get_value(val, true);
      benchmark::DoNotOptimize(val);
    }
    for (const auto & h : command_handles_)
    {
      std::ignore = h->get_value(val, true);
      benchmark::DoNotOptimize(val);
    }
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_WriteHandleDirect)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (auto & h : state_handles_)
    {
      std::ignore = h->set_value(val, true);
    }
    for (auto & h : command_handles_)
    {
      std::ignore = h->set_value(val, true);
    }
    benchmark::ClobberMemory();
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_ReadMapLookup)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  // Build name vectors for lookup
  std::vector<std::string> state_names;
  std::vector<std::string> command_names;
  for (const auto & [name, _] : state_handle_map_)
  {
    state_names.push_back(name);
  }
  for (const auto & [name, _] : command_handle_map_)
  {
    command_names.push_back(name);
  }

  double val = 0.0;
  for (auto _ : state)
  {
    for (const auto & name : state_names)
    {
      auto it = state_handle_map_.find(name);
      if (it != state_handle_map_.end())
      {
        std::ignore = it->second->get_value(val, true);
        benchmark::DoNotOptimize(val);
      }
    }
    for (const auto & name : command_names)
    {
      auto it = command_handle_map_.find(name);
      if (it != command_handle_map_.end())
      {
        std::ignore = it->second->get_value(val, true);
        benchmark::DoNotOptimize(val);
      }
    }
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(PrimitiveBenchmarkFixture, BM_Prim_WriteMapLookup)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  std::vector<std::string> state_names;
  std::vector<std::string> command_names;
  for (const auto & [name, _] : state_handle_map_)
  {
    state_names.push_back(name);
  }
  for (const auto & [name, _] : command_handle_map_)
  {
    command_names.push_back(name);
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (const auto & name : state_names)
    {
      auto it = state_handle_map_.find(name);
      if (it != state_handle_map_.end())
      {
        std::ignore = it->second->set_value(val, true);
      }
    }
    for (const auto & name : command_names)
    {
      auto it = command_handle_map_.find(name);
      if (it != command_handle_map_.end())
      {
        std::ignore = it->second->set_value(val, true);
      }
    }
    benchmark::ClobberMemory();
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

// Register Tier 1 benchmarks
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_ReadRawPointer)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_WriteRawPointer)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_ReadHandleDirect)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_WriteHandleDirect)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_ReadMapLookup)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(PrimitiveBenchmarkFixture, BM_Prim_WriteMapLookup)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);

// ============================================================================
// Tier 2: API Benchmark Fixture (through HardwareComponent/System wrapper)
// ============================================================================

class APIBenchmarkFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    setup_error_.clear();

    const size_t scenario_idx = static_cast<size_t>(state.range(0));
    if (scenario_idx >= kNumScenarios)
    {
      setup_error_ = "Invalid scenario index";
      g_setup_failed.store(true);
      return;
    }

    const auto & scenario = kScenarios[scenario_idx];
    num_state_ = scenario.num_state;
    num_command_ = scenario.num_command;

    // Build HardwareInfo
    auto hw_info = build_hardware_info(num_state_, num_command_);

    // Create System with BenchmarkSystem, keeping a raw pointer for string-based API access
    auto benchmark_system = std::make_unique<BenchmarkSystem>();
    hw_interface_ = benchmark_system.get();  // Store raw pointer before move
    system_ = std::make_unique<hardware_interface::System>(std::move(benchmark_system));

    // Initialize through lifecycle
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hw_info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    try
    {
      auto init_state = system_->initialize(params);
      if (init_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      {
        setup_error_ = "Failed to initialize system";
        g_setup_failed.store(true);
        return;
      }

      auto config_state = system_->configure();
      if (config_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        setup_error_ = "Failed to configure system";
        g_setup_failed.store(true);
        return;
      }

      // Export interfaces
      auto state_interfaces = system_->export_state_interfaces();
      auto command_interfaces = system_->export_command_interfaces();

      // Build name lists
      state_names_.clear();
      command_names_.clear();
      for (const auto & si : state_interfaces)
      {
        state_names_.push_back(si->get_name());
      }
      for (const auto & ci : command_interfaces)
      {
        command_names_.push_back(ci->get_name());
      }

      // Validate counts
      if (state_names_.size() != num_state_)
      {
        setup_error_ = "State interface count mismatch: expected " +
                       std::to_string(num_state_) + ", got " +
                       std::to_string(state_names_.size());
        g_setup_failed.store(true);
        return;
      }
      if (command_names_.size() != num_command_)
      {
        setup_error_ = "Command interface count mismatch: expected " +
                       std::to_string(num_command_) + ", got " +
                       std::to_string(command_names_.size());
        g_setup_failed.store(true);
        return;
      }

      // Cache handles via the real HardwareComponentInterface API (#2831)
      cached_state_handles_.clear();
      cached_command_handles_.clear();
      for (const auto & name : state_names_)
      {
        cached_state_handles_.push_back(hw_interface_->get_state_interface_handle(name));
      }
      for (const auto & name : command_names_)
      {
        cached_command_handles_.push_back(hw_interface_->get_command_interface_handle(name));
      }

      // Round-trip probe validation using real HardwareComponentInterface API
      const double sentinel = 42.0;
      for (size_t i = 0; i < cached_state_handles_.size(); ++i)
      {
        if (!hw_interface_->set_state(cached_state_handles_[i], sentinel, true))
        {
          setup_error_ = "set_state(cached) failed at " + state_names_[i];
          g_setup_failed.store(true);
          return;
        }
        double readback = 0.0;
        if (!hw_interface_->get_state(cached_state_handles_[i], readback, true) ||
            readback != sentinel)
        {
          setup_error_ = "state round-trip failed at " + state_names_[i];
          g_setup_failed.store(true);
          return;
        }
      }
      for (size_t i = 0; i < cached_command_handles_.size(); ++i)
      {
        if (!hw_interface_->set_command(cached_command_handles_[i], sentinel, true))
        {
          setup_error_ = "set_command(cached) failed at " + command_names_[i];
          g_setup_failed.store(true);
          return;
        }
        double readback = 0.0;
        if (!hw_interface_->get_command(cached_command_handles_[i], readback, true) ||
            readback != sentinel)
        {
          setup_error_ = "command round-trip failed at " + command_names_[i];
          g_setup_failed.store(true);
          return;
        }
      }

      // Create shuffled name orders for handle lookup benchmarks
      shuffled_state_names_ = state_names_;
      shuffled_command_names_ = command_names_;
      std::mt19937 rng(42);  // Deterministic seed
      std::shuffle(shuffled_state_names_.begin(), shuffled_state_names_.end(), rng);
      std::shuffle(shuffled_command_names_.begin(), shuffled_command_names_.end(), rng);

    }
    catch (const std::exception & e)
    {
      setup_error_ = std::string("Setup exception: ") + e.what();
      g_setup_failed.store(true);
      return;
    }
  }

  void TearDown(benchmark::State & /*state*/) override
  {
    cached_state_handles_.clear();
    cached_command_handles_.clear();
    state_names_.clear();
    command_names_.clear();
    shuffled_state_names_.clear();
    shuffled_command_names_.clear();
    hw_interface_ = nullptr;
    system_.reset();
  }

protected:
  std::string setup_error_;
  size_t num_state_ = 0;
  size_t num_command_ = 0;

  std::unique_ptr<hardware_interface::System> system_;
  hardware_interface::HardwareComponentInterface * hw_interface_ = nullptr;  // For string-based API
  std::vector<std::string> state_names_;
  std::vector<std::string> command_names_;
  std::vector<std::string> shuffled_state_names_;
  std::vector<std::string> shuffled_command_names_;
  std::vector<hardware_interface::StateInterface::SharedPtr> cached_state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> cached_command_handles_;
};

// ============================================================================
// Tier 2 Benchmarks: Public API (Handle-based access)
// ============================================================================

// Cached-handle benchmarks call HardwareComponentInterface::get_state/set_state/get_command/
// set_command with pre-acquired handles (PR #2831 path). All calls go through hw_interface_.

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_GetStateCached)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double val = 0.0;
  for (auto _ : state)
  {
    for (const auto & h : cached_state_handles_)
    {
      std::ignore = hw_interface_->get_state(h, val, true);
      benchmark::DoNotOptimize(val);
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_SetStateCached)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (const auto & h : cached_state_handles_)
    {
      std::ignore = hw_interface_->set_state(h, val, true);
    }
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_GetCommandCached)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double val = 0.0;
  for (auto _ : state)
  {
    for (const auto & h : cached_command_handles_)
    {
      std::ignore = hw_interface_->get_command(h, val, true);
      benchmark::DoNotOptimize(val);
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_SetCommandCached)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (const auto & h : cached_command_handles_)
    {
      std::ignore = hw_interface_->set_command(h, val, true);
    }
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_FullCycleCached)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  double val = 0.0;
  const double write_val = 42.0;
  for (auto _ : state)
  {
    // Read all states
    for (const auto & h : cached_state_handles_)
    {
      std::ignore = hw_interface_->get_state(h, val, true);
      benchmark::DoNotOptimize(val);
    }
    // Write all commands
    for (const auto & h : cached_command_handles_)
    {
      std::ignore = hw_interface_->set_command(h, write_val, true);
    }
    benchmark::ClobberMemory();
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

// ============================================================================
// Tier 2 Benchmarks: Public API (String-based access - pre-#2831 pattern)
// ============================================================================

// These benchmarks measure the string-based lookup pattern that was the default
// before PR #2831. Each access requires a map lookup by name.

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_GetStateString)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : state_names_)
    {
      auto val = hw_interface_->get_state<double>(name);
      benchmark::DoNotOptimize(val);
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_SetStateString)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (const auto & name : state_names_)
    {
      hw_interface_->set_state<double>(name, val);
    }
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_GetCommandString)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : command_names_)
    {
      auto val = hw_interface_->get_command<double>(name);
      benchmark::DoNotOptimize(val);
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_SetCommandString)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double val = 42.0;
  for (auto _ : state)
  {
    for (const auto & name : command_names_)
    {
      hw_interface_->set_command<double>(name, val);
    }
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(APIBenchmarkFixture, BM_API_FullCycleString)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  const double write_val = 42.0;
  for (auto _ : state)
  {
    // Read all states via string-based API
    for (const auto & name : state_names_)
    {
      auto val = hw_interface_->get_state<double>(name);
      benchmark::DoNotOptimize(val);
    }
    // Write all commands via string-based API
    for (const auto & name : command_names_)
    {
      hw_interface_->set_command<double>(name, write_val);
    }
    benchmark::ClobberMemory();
  }

  const size_t total = num_state_ + num_command_;
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(total));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * total),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

// Register Tier 2 benchmarks (Cached - PR #2831 pattern)
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_GetStateCached)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_SetStateCached)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_GetCommandCached)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_SetCommandCached)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_FullCycleCached)
  ->DenseRange(1, kNumScenarios - 1)  // Skip smallest scenario for full cycle
  ->Unit(benchmark::kNanosecond);

// Register Tier 2 benchmarks (String-based - pre-#2831 pattern)
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_GetStateString)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_SetStateString)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_GetCommandString)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_SetCommandString)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(APIBenchmarkFixture, BM_API_FullCycleString)
  ->DenseRange(1, kNumScenarios - 1)  // Skip smallest scenario for full cycle
  ->Unit(benchmark::kNanosecond);

// ============================================================================
// Minimal Fixture for Handle Lookup Benchmarks (no pre-caching)
// ============================================================================

class HandleLookupFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    setup_error_.clear();

    const size_t scenario_idx = static_cast<size_t>(state.range(0));
    if (scenario_idx >= kNumScenarios)
    {
      setup_error_ = "Invalid scenario index";
      g_setup_failed.store(true);
      return;
    }

    const auto & scenario = kScenarios[scenario_idx];
    num_state_ = scenario.num_state;
    num_command_ = scenario.num_command;

    auto hw_info = build_hardware_info(num_state_, num_command_);

    auto benchmark_system = std::make_unique<BenchmarkSystem>();
    hw_interface_ = benchmark_system.get();
    system_ = std::make_unique<hardware_interface::System>(std::move(benchmark_system));

    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hw_info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    try
    {
      system_->initialize(params);
      system_->configure();

      auto state_interfaces = system_->export_state_interfaces();
      auto command_interfaces = system_->export_command_interfaces();

      // Build name lists only - NO handle caching
      state_names_.clear();
      command_names_.clear();
      for (const auto & si : state_interfaces)
      {
        state_names_.push_back(si->get_name());
      }
      for (const auto & ci : command_interfaces)
      {
        command_names_.push_back(ci->get_name());
      }

      // Create shuffled orders
      shuffled_state_names_ = state_names_;
      shuffled_command_names_ = command_names_;
      std::mt19937 rng(42);
      std::shuffle(shuffled_state_names_.begin(), shuffled_state_names_.end(), rng);
      std::shuffle(shuffled_command_names_.begin(), shuffled_command_names_.end(), rng);

    }
    catch (const std::exception & e)
    {
      setup_error_ = std::string("Setup exception: ") + e.what();
      g_setup_failed.store(true);
    }
  }

  void TearDown(benchmark::State & /*state*/) override
  {
    state_names_.clear();
    command_names_.clear();
    shuffled_state_names_.clear();
    shuffled_command_names_.clear();
    hw_interface_ = nullptr;
    system_.reset();
  }

protected:
  std::string setup_error_;
  size_t num_state_ = 0;
  size_t num_command_ = 0;

  std::unique_ptr<hardware_interface::System> system_;
  hardware_interface::HardwareComponentInterface * hw_interface_ = nullptr;
  std::vector<std::string> state_names_;
  std::vector<std::string> command_names_;
  std::vector<std::string> shuffled_state_names_;
  std::vector<std::string> shuffled_command_names_;
};

// Handle lookup benchmarks - measures map lookup cost (steady-state throughput)
BENCHMARK_DEFINE_F(HandleLookupFixture, BM_API_AcquireStateHandle)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : state_names_)
    {
      const auto & h = hw_interface_->get_state_interface_handle(name);
      benchmark::DoNotOptimize(h.get());
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
}

BENCHMARK_DEFINE_F(HandleLookupFixture, BM_API_AcquireStateHandleShuffled)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : shuffled_state_names_)
    {
      const auto & h = hw_interface_->get_state_interface_handle(name);
      benchmark::DoNotOptimize(h.get());
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_state_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_state_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_state_interfaces"] = static_cast<double>(num_state_);
}

BENCHMARK_DEFINE_F(HandleLookupFixture, BM_API_AcquireCommandHandle)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : command_names_)
    {
      const auto & h = hw_interface_->get_command_interface_handle(name);
      benchmark::DoNotOptimize(h.get());
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

BENCHMARK_DEFINE_F(HandleLookupFixture, BM_API_AcquireCommandHandleShuffled)(benchmark::State & state)
{
  if (!setup_error_.empty())
  {
    state.SkipWithError(setup_error_.c_str());
    return;
  }

  for (auto _ : state)
  {
    for (const auto & name : shuffled_command_names_)
    {
      const auto & h = hw_interface_->get_command_interface_handle(name);
      benchmark::DoNotOptimize(h.get());
    }
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_command_));
  state.counters["ns_per_interface"] = benchmark::Counter(
    static_cast<double>(state.iterations() * num_command_),
    benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["num_command_interfaces"] = static_cast<double>(num_command_);
}

// Register handle lookup benchmarks
BENCHMARK_REGISTER_F(HandleLookupFixture, BM_API_AcquireStateHandle)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(HandleLookupFixture, BM_API_AcquireStateHandleShuffled)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(HandleLookupFixture, BM_API_AcquireCommandHandle)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(HandleLookupFixture, BM_API_AcquireCommandHandleShuffled)
  ->DenseRange(0, kNumScenarios - 1)
  ->Unit(benchmark::kNanosecond);

}  // namespace

// ============================================================================
// Main
// ============================================================================

int main(int argc, char ** argv)
{
  // Process-wide rclcpp init - no args to avoid conflict with benchmark arg parsing
  rclcpp::init(0, nullptr);

  benchmark::Initialize(&argc, argv);

  int ret = 0;
  if (benchmark::ReportUnrecognizedArguments(argc, argv))
  {
    std::cerr << "benchmark_handle_access: unrecognized arguments\n";
    ret = 1;
  }
  else
  {
    benchmark::RunSpecifiedBenchmarks();
  }

  benchmark::Shutdown();
  rclcpp::shutdown();

  return ret != 0 ? ret : (g_setup_failed.load() ? 1 : 0);
}
