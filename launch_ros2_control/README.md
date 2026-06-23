# launch_ros2_control
A launch extension for [ros2_control](control.ros.org).

This packages provides a launch action for spawning controllers.
It is compatible with Humble, Jazzy, Kilted and Rolling.

Examples for the <i title="XML, YAML, and Python">3 major launch file types</i>, can be found in the [examples folder](examples/).

## <span title="XML/YAML tag">`spawn_contoller`/`spawn-controller`</span> (<span title="Internal Python Action 🐍">`SpawnControllers`</span>)
The `spawn_controller` element can be used to spawn multiple controllers.
It has the following attributes:

### Attributes
The format is as follows: (format is hoverable.)
> <span title="The name or key of the attribute.">`<attr_name>`</span> (<i title="If the attribute is optional.">\<?OPTIONAL\></i>, <span title="The type of the attribute. Marked as `Substitution` if a substitutions are allowed, which should resolve to `<TARGET_TYPE>`">TYPE/Substitution\[\<REVOLVED_TYPE\>\]</span>): Description (<span title="When an attribute is marked with 'spawner default' it means the default spawner behavior is used.">\<?spawner default\></span>)

 - `spawner_name` (Optional, Substitution\[string\]): The node name for the spawner. Defaults to the `controller_manager/spawner` default naming (spawner default).
 - `controller_manager` (Optional, Substitution\[string\]): The name of the target controller manager node. Defaults to `'controller_manager'` (spawner default).
 - `final_controller_state` (Optional, Substitution\[string\]): The state for all controllers to end in, can be either `'active'`, `'inactive'` or `'unconfigured'`. Defaults to `'active'`.
 - `unload_on_kill` (Optional, Substitution\[bool\]): If the spawner should stay active until the launch file receives a shutdown signal, at which moment the controllers will be unloaded. Defaults to `False`.
 - `activate_as_group` (Optional, Substitution\[bool\]): If all controllers should be activated at once. Useful when activating chained controllers together. Defaults to `False`.
 - `controller_manager_timeout` (Optional, Substitution\[float\]): Time to wait for the controller manager service to be available. Defaults to `0.0s` (indefinite) (spawner default).
 - `switch_timeout` (Optional, Substitution\[float\]): Time to wait for a successful state switch of controllers. Useful when switching cannot be performed immediately, e.g., paused simulations at startup. Defaults to `5.0s` (spawner default).
 - `service_call_timeout` (Optional, Substitution\[float\]): Time to wait for the service response from the controller manager. Defaults to `10.0s` (spawner default).
 - `emulate_tty` (Optional, Substitution\[bool\]): Passed to the internally used Node action's `emulate_tty` attribute. Defaults to `False`.
 - `output` (Optional, Substitution\[string\]): Passed to the internally used Node action's `output` attribute, commonly used options are <i title="Log stdout and stderr to the screen">`'screen'`</i>, <i title="Log stdout and stderr to a shared log file">`'log'`</i> and <i title="Log stdout and stderr to the screen and a shared log file">`'both'`</i>, but other options are available. Defaults to `'log'`.
 - `if`/`unless` (Optional, Substitution\[bool\]): The standard action conditionals. Always active if omitted.

### Subelements/Structures
The `spawn_controller` action can have a list of `controller` elements to spawn.
The controller elements are structured as follows:

#### Controller attributes
- `name` (Substitution\[string\]): The name of the controller to spawn.
- `if`/`unless` (Optional, Substitution\[bool\]): The standard conditionals. Always active if omitted.

#### Controller subelements
- `remap` (Optional, zero or more): The same remap structure as available on a `node` action, however these get specified with the controller name, so only this specific controller is effected.
  - `from` (Substitution\[string\]): Which topic/service/action to remap.[^1]
  - `to` (Substitution\[string\]): The target to remap to.
- `param` (Optional, zero or more): The same parameter structure as available on the `node` action, however these get applied to the specific controller[^2]. If normal parameters are provided for a controller its `type` parameter should also be specified. See the `node` action for details.

[^1]: Action remapping is [broken in Humble](https://github.com/ros2/ros2/issues/1312). The fix is available since Kilted (Kilted and up), however [the Jazzy backport is still pending](https://github.com/ros2/rcl/pull/1220).

[^2]: Unless a parameter file is loaded, then it uses the specifier in the controller file.
