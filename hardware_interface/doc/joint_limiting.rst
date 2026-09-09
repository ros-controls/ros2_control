:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/joint_limiting.rst

.. _joint_limiting:


Joint Limiting for ros2_control
---------------------------------------------------------

ros2_control provides several mechanisms to handle joint limits for different hardware interfaces.

Enable Joint Limits
#####################

There exists several ways in controlling the joint limits handling in ros2_control:

* globally for all interfaces of all hardware components via the ``ros2_control_node`` parameter ``enforce_command_limits``, for details see :ref:`here <doc/ros2_control/controller_manager/doc/userdoc:parameters>`.
* for all interfaces of a joint

  .. code-block:: xml

      <joint name="joint1">
        ...
        <limits enable="false"/>
        ...
      </joint>
* for a single interface of a joint

  .. code-block:: xml

      <joint name="joint1">
        ...
        <command_interface name="position">
          <limits enable="false"/>
        </command_interface>
        ...
      </joint>

If joint limits are active for a specific interface, the controller_manger will print a similar message on startup:

  .. code-block:: text

    [controller_manager]: Using JointLimiter for joint 'joint1' in hardware 'RRBot' : '  has position limits: true [-1, 1]
    [ros2_control_node-1]   has velocity limits: true [1]
    [ros2_control_node-1]   has acceleration limits: false [nan]
    [ros2_control_node-1]   has deceleration limits: false [nan]
    [ros2_control_node-1]   has jerk limits: false [nan]
    [ros2_control_node-1]   has effort limits: true [100]
    [ros2_control_node-1]   angle wraparound: true'

Configuration of Limits
########################

Joint limits can be configured through parameters on the ROS 2 node. The limit parameters follow a standard naming convention and structure.

Basic Parameter Structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~

For each joint, limits are organized under the ``joint_limits.<joint_name>`` namespace with the following structure:

.. code-block:: yaml

   joint_limits:
     <joint_name>:
       # Position Limits
       has_position_limits: bool         # Enable/disable position limits
       min_position: double              # Minimum position [rad or m]
       max_position: double              # Maximum position [rad or m]
       # Velocity Limits
       has_velocity_limits: bool         # Enable/disable velocity limits
       max_velocity: double              # Maximum absolute velocity [rad/s or m/s]
       # Acceleration Limits
       has_acceleration_limits: bool     # Enable/disable acceleration limits
       max_acceleration: double          # Maximum acceleration [rad/s² or m/s²]
       # Deceleration Limits
       has_deceleration_limits: bool     # Enable/disable deceleration limits
       max_deceleration: double          # Maximum deceleration [rad/s² or m/s²]
       # Jerk Limits (rate of change of acceleration)
       has_jerk_limits: bool             # Enable/disable jerk limits
       max_jerk: double                  # Maximum jerk [rad/s³ or m/s³]
       # Effort Limits
       has_effort_limits: bool           # Enable/disable effort/force limits
       max_effort: double                # Maximum effort/force [N·m or N]
       # Angle Wraparound (for continuous joints)
       angle_wraparound: bool            # Enable angle wrapping for continuous joints [default: false]
       # Soft Limits (optional, for gradual limiting)
       has_soft_limits: bool             # Enable/disable soft limits [default: false]
       k_position: double                # Spring constant for position [dimensionless]
       k_velocity: double                # Damping coefficient for velocity [dimensionless]
       soft_lower_limit: double          # Soft limit lower boundary [rad or m]
       soft_upper_limit: double          # Soft limit upper boundary [rad or m]

Configuration Methods
~~~~~~~~~~~~~~~~~~~~~~~

**1. Via Parameter File (YAML)**

Create a parameter file (e.g., ``joint_limits.yaml``) and pass it to the ``ros2_control_node``:

.. code-block:: yaml

   ros2_control_node:
     ros__parameters:
       enforce_command_limits: true
   joint_limits:
     joint1:
       has_position_limits: true
       min_position: -1.57
       max_position: 1.57
       has_velocity_limits: true
       max_velocity: 1.0
       has_acceleration_limits: true
       max_acceleration: 2.0
       has_effort_limits: true
       max_effort: 100.0
     joint2:
       has_position_limits: false
       has_velocity_limits: true
       max_velocity: 2.0
       angle_wraparound: true

**2. Via URDF (Hardware Configuration)**

Hard limits are defined in the standard URDF ``<limit>`` tag:

.. code-block:: xml

   <ros2_control name="RRBot" type="system">
     <hardware>
       <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
     </hardware>
     <joint name="joint1" type="revolute">
       <command_interface name="position"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
       <limit lower="-1.57" upper="1.57" velocity="1.0" effort="100.0"/>
     </joint>
     <joint name="joint2" type="continuous">
       <command_interface name="velocity"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
       <limit velocity="2.0" effort="50.0"/>
     </joint>
   </ros2_control>

**3. Soft Limits via URDF**

Soft limits are specified using the ``<safety_controller>`` tag within the ``<joint>`` element:

.. code-block:: xml

   <joint name="joint1" type="revolute">
     <command_interface name="position"/>
     <state_interface name="position"/>
     <state_interface name="velocity"/>
     <!-- Hard limits -->
     <limit lower="-1.57" upper="1.57" velocity="1.0" effort="100.0"/>
     <!-- Soft limits (optional) -->
     <safety_controller
       k_position="10.0"
       k_velocity="20.0"
       soft_lower_limit="-1.40"
       soft_upper_limit="1.40"/>
   </joint>

**4. Soft Limits via Parameters**

Alternatively, configure soft limits via YAML:

.. code-block:: yaml

   joint_limits:
     joint1:
       has_position_limits: true
       min_position: -1.57
       max_position: 1.57
       # Soft limits trigger before hard limits
       has_soft_limits: true
       soft_lower_limit: -1.40
       soft_upper_limit: 1.40
       # Spring-damper parameters
       k_position: 10.0      # Position spring constant (higher = stiffer spring)
       k_velocity: 20.0      # Velocity damping coefficient

**5. Disabling Limits for Specific Interfaces**

You can selectively disable limits for specific command interfaces:

.. code-block:: xml

   <joint name="joint1" type="revolute">
     <command_interface name="position">
       <limits enable="false"/>  <!-- Position commands not limited -->
     </command_interface>
     <command_interface name="velocity"/>  <!-- Velocity commands are limited -->
     <state_interface name="position"/>
     <state_interface name="velocity"/>
   </joint>

Description of the Limiter Algorithms
######################################

The ros2_control framework implements two primary joint limiting strategies: **Saturation Limiting** (default) and **Soft Limiting** (optional).

Saturation Limiter (Default)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Saturation Limiter uses clamping to enforce hard limits on joint commands. It considers the interdependencies between position, velocity, and acceleration to ensure physically feasible commands.

**Algorithm Overview:**

1. **Position Limiting**: Clamps desired position to hard limits ``[min_position, max_position]``

2. **Velocity Limiting**: Limits velocity based on:
   - Hard velocity limits: ``max_velocity``
   - Position proximity: As the joint approaches its position limits, the maximum allowed velocity decreases
   - Acceleration capabilities: ``velocity_limit = min(max_velocity, current_velocity + max_acceleration * dt)``

3. **Acceleration Limiting**: Constrains acceleration/deceleration:
   - Detects if the joint is accelerating (velocity increasing) or decelerating (velocity approaching zero)
   - Applies ``max_acceleration`` or ``max_deceleration`` accordingly
   - Recomputes position and velocity to maintain consistency

4. **Effort Limiting**: Clamps effort commands while respecting boundary conditions:
   - At lower position limit with downward velocity: only positive (upward) effort allowed
   - At upper position limit with upward velocity: only negative (downward) effort allowed
   - Prevents effort that would push joint further out of bounds

5. **Jerk Limiting**: Clamps jerk (rate of change of acceleration) to ``[-max_jerk, max_jerk]``

**Enforcement Priority:**

The limiter processes constraints in order, with later stages recomputing values based on earlier constraints:

.. code-block:: text

   Position Limits
        ↓
   Velocity Limits (adjusted by position proximity)
        ↓
   Acceleration/Deceleration Limits
        ↓
   Effort Limits (respecting boundary conditions)
        ↓
   Jerk Limits

**Example Scenario:**

A joint approaching its upper position limit will have its velocity limit progressively reduced to ensure it can decelerate safely:

.. code-block:: text

   Position: -1.0 to 1.0 (rad)
   Velocity limit: 1.0 (rad/s)
   Acceleration limit: 2.0 (rad/s²)
   Current position: 0.95 (approaching limit at 1.0)
   Current velocity: 0.5 (rad/s)
   Available distance to limit: 1.0 - 0.95 = 0.05 rad
   Maximum safe velocity: 0.05 rad / dt
   If dt = 0.01s, max_safe_velocity = 5.0 rad/s (no adjustment needed)
   If dt = 0.1s, max_safe_velocity = 0.5 rad/s (velocity already OK)
   If dt = 1.0s, max_safe_velocity = 0.05 rad/s (limit velocity!)

Soft Limiter
~~~~~~~~~~~~

The Soft Limiter implements a **spring-damper model** for gradual limiting instead of hard clamping. This provides smoother behavior and can prevent abrupt changes in joint behavior, especially useful for safety-critical applications.

**Spring-Damper Model:**

The soft limits create a "virtual spring" that gradually increases resistance as the joint approaches the hard limit. The velocity is reduced proportionally to the distance from the soft limit boundary:

.. code-block:: text

   For lower soft limit:
   v_limit_min = -k_position * (current_position - soft_lower_limit)
   For upper soft limit:
   v_limit_max = -k_position * (current_position - soft_upper_limit)
   Where:
   - k_position: Spring constant (higher = stronger braking force)
   - current_position: Current joint position
   - soft_limit_position: The soft limit boundary

**Sign Convention:**

- When position exceeds upper soft limit (position > soft_upper): negative velocity limit applied (must move downward)
- When position falls below lower soft limit (position < soft_lower): positive velocity limit applied (must move upward)
- Uses previous command position for stability (reduces feedback lag)

**Algorithm Phases:**

1. **Normal Operation Zone** (between soft limits):
   - Joint moves freely within hard velocity limits
   - No soft limit effects applied

2. **Soft Limit Zone** (between soft and hard limits):
   - Virtual spring activates and applies proportional braking
   - Velocity is gradually reduced based on distance from soft limit
   - Damping term ``k_velocity`` prevents oscillation and provides smooth deceleration
   - Minimum velocity (≈0.017 rad/s or 1°/s) maintained for controlled approach to hard limit

3. **Hard Limit Zone** (beyond hard limits or at boundary):
   - Velocity is restricted to prevent exceeding hard limits
   - Joint can only move back toward soft limits
   - Effort limits prevent pushing further out of bounds

**Example:**

.. code-block:: text

   Hard limits: [-1.57, 1.57]
   Soft limits: [-1.40, 1.40]
   k_position: 10.0
   k_velocity: 20.0
   Position = 1.45 (in soft zone, approaching upper limit)
   Calculation:
   v_limit_max = -10.0 * (1.45 - 1.40)
   v_limit_max = -10.0 * 0.05 = -0.5 rad/s
   Result: Joint velocity is limited to -0.5 rad/s (moving downward/backward)
   This gives the joint time to decelerate before reaching hard limit at 1.57

**Soft Limits vs Hard Limits:**

- **Soft Limits**: Gradual, spring-like braking that feels smooth to operators
- **Hard Limits**: Immediate clamping that can feel abrupt but guarantees hard bounds

Use soft limits for human-robot interaction and smooth motion profiles.

Dual Enforcement Strategy
~~~~~~~~~~~~~~~~~~~~~~~~~~

Joint limits are enforced at **two points** in the control loop:

1. **Command Interface Level**: When a controller sets a command value via ``set_value()``:
   - The limiter immediately constrains the command
   - ``is_limited()`` flag indicates if limiting was applied
   - Command Interface reflects the limited value
   - Happens before the control loop update

2. **Controller Manager Update Level**: At the end of the update cycle:
   - Final enforcement ensures coherence across multiple interfaces
   - Example: If both position and velocity commands exist, limits ensure consistency
   - Prevents race conditions between different control interfaces
   - Applies additional limiting if needed

This dual approach guarantees limits are never violated, even with complex multi-interface scenarios involving feedback delays.

Thread Safety
~~~~~~~~~~~~~

Joint limits are thread-safe and support **dynamic reconfiguration**:

- Limits can be changed at runtime via parameter server updates
- Real-time safe: Uses ``RealtimeBuffer`` for zero-copy parameter access (lock-free reading in control loop)
- Safe parameter updates from the parameter server using callback mechanism
- No blocking operations in the control loop

Best Practices
~~~~~~~~~~~~~~

1. **Set Conservative Soft Limits**: Place soft limits 5-10% before hard limits to give the controller time to react
   Example: Hard limits [-1.57, 1.57] → Soft limits [-1.40, 1.40]

2. **Match Acceleration to System Capabilities**: Set ``max_acceleration`` and ``max_deceleration`` to achievable values
   - Too low: Sluggish motion, excessive limiting
   - Too high: Risk of not meeting hard limits in emergency

3. **Use Soft Limits for User Safety**: Apply soft limits in applications where sudden stops could be problematic
   - Collaborative robots (cobots)
   - Force-feedback systems
   - Sensitive machinery

4. **Enable Velocity Limits**: Always enable velocity limits for safety, especially for high-mass robots

5. **Tune Spring Constant (k_position)**: Higher values = earlier and stronger braking
   - Conservative: k_position = 5-10
   - Balanced: k_position = 10-20
   - Aggressive: k_position = 20+

6. **Monitor Logged Output**: Watch controller_manager startup messages to confirm limits are loaded correctly

7. **Test Before Deployment**: Verify joint behavior with configured limits using simulation before real hardware
