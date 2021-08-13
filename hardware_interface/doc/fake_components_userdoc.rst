.. _fake_components_userdoc:

Fake Components
----------------
Fake components are trivial "simulations" of the hardware components, i.e., System, Sensor, and Actuator.
They provide ideal behavior by mirroring commands to their states.
The corresponding hardware interface can be added instead of real hardware for offline testing of ros2_control framework.
The main advantage is that you can test all the "piping" inside the framework without having access to the hardware.
This means that you can test your controllers, broadcaster, launch files, and even integrations with, e.g., MoveIt.
The main intention is to reduce debugging time on the physical hardware and boost your development.


Generic System
^^^^^^^^^^^^^^
The component implements ``hardware_interface::SystemInterface>`` supporting command and state interfaces.
For more information about hardware components check :ref:`detailed documentation <overview_hardware_components>`.

Features:

  - support for mimic joints
  - mirroring commands to states with and without offset
  - fake command interfaces for setting sensor data from an external node (combined with a :ref:`forward controller <forward_command_controller_userdoc>`)


Parameters
,,,,,,,,,,

fake_sensor_commands (optional; boolean; default: false)
  Creates fake command interfaces for faking sensor measurements with an external command.
  Those interfaces are usually used by a :ref:`forward controller <forward_command_controller_userdoc>` to provide access from ROS-world.

position_state_following_offset (optional; double; default: 0.0)
  Following offset added to the commanded values when mirrored to states.


custom_interface_with_following_offset (optional; string; default: "")
  Mapping of offsetted commands to a custom interface.


Per-joint Parameters
,,,,,,,,,,,,,,,,,,,,

mimic (optional; string)
  Defined name of the joint to mimic. This is often used concept with parallel grippers. Example: ``<param name="mimic">joint1</param>``.


multiplier (optional; double; default: 1; used if mimic joint is defined)
  Multiplier of values for mimicking joint defined in ``mimic`` parameter. Example: ``<param name="multiplier">-2</param>``.
