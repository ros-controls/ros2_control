# Copyright 2026 Levin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

_TRANSITIONS = {
    ("unconfigured", "inactive"): ("configure",),
    ("unconfigured", "active"): ("configure", "activate"),
    ("inactive", "unconfigured"): ("cleanup",),
    ("inactive", "active"): ("activate",),
    ("active", "unconfigured"): ("deactivate", "cleanup"),
    ("active", "inactive"): ("deactivate",),
}
_PRIMARY_STATES = {"unconfigured", "inactive", "active"}


def plan_state_transitions(current_state, target_state):
    """Return lifecycle transitions needed to reach a primary target state."""
    if current_state not in _PRIMARY_STATES:
        raise ValueError(f"unsupported current controller state: {current_state}")
    if target_state not in _PRIMARY_STATES:
        raise ValueError(f"unsupported target controller state: {target_state}")
    if current_state == target_state:
        return []
    return list(_TRANSITIONS[(current_state, target_state)])
