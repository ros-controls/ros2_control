// Copyright 2022 PickNik Inc.
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

#include "controller_manager/realtime.hpp"

#include <sched.h>

#include <cstring>
#include <fstream>

namespace controller_manager
{
bool has_realtime_kernel()
{
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open())
  {
    realtime_file >> has_realtime;
  }
  return has_realtime;
}

bool configure_sched_fifo(int priority)
{
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &schedp))
  {
    return false;
  }
  return true;
}

}  // namespace controller_manager
