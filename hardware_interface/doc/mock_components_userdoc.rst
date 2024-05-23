:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/mock_components_userdoc.rst

.. _mock_components_userdoc:

Mock Components
----------------
Mock components are trivial "simulations" of the hardware components, i.e., System, Sensor, and Actuator.
They provide ideal behavior by mirroring commands to their states.
The corresponding hardware interface can be added instead of real hardware for offline testing of ros2_control framework.
The main advantage is that you can test all the "piping" inside the framework without having access to the hardware.
This means that you can test your controllers, broadcaster, launch files, and even integrations with, e.g., MoveIt.
The main intention is to reduce debugging time on the physical hardware and boost your development.


Generic System
^^^^^^^^^^^^^^
The component implements ``hardware_interface::SystemInterface`` supporting command and state interfaces.
For more information about hardware components check :ref:`detailed documentation <overview_hardware_components>`.

Features:

  - support for mimic joints, which is parsed from the URDF (see the `URDF wiki <http://wiki.ros.org/urdf/XML/joint>`__)
  - mirroring commands to states with and without offset
  - fake command interfaces for setting sensor data from an external node (combined with a :ref:`forward controller <forward_command_controller_userdoc>`)
  - fake gpio interfaces for setting sensor data from an external node (combined with a :ref:`forward controller <forward_command_controller_userdoc>`)


Parameters
,,,,,,,,,,

A full example including all optional parameters (with default values):

.. code-block:: xml

  <ros2_control name="MockHardwareSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">false</param>
      <param name="custom_interface_with_following_offset"></param>
      <param name="disable_commands">false</param>
      <param name="mock_gpio_commands">false</param>
      <param name="mock_sensor_commands">false</param>
      <param name="position_state_following_offset">0.0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">3.45</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <command_interface name="acceleration"/>
      <state_interface name="position">
        <param name="initial_value">2.78</param>
      </state_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <gpio name="flange_vacuum">
      <command_interface name="vacuum"/>
      <state_interface name="vacuum" data_type="double"/>
    </gpio>
  </ros2_control>

See :ref:`example_2 <ros2_control_demos_example_2_userdoc>` for an example using ``calculate_dynamics`` or :ref:`example_10 <ros2_control_demos_example_10_userdoc>` for using in combination with GPIO interfaces.

Component Parameters
####################

calculate_dynamics (optional; boolean; default: false)
  Calculation of states from commands by using Euler-forward integration or finite differences.

custom_interface_with_following_offset (optional; string; default: "")
  Mapping of offsetted commands to a custom interface.

disable_commands (optional; boolean; default: false)
  Disables mirroring commands to states.
  This option is helpful to simulate an erroneous connection to the hardware when nothing breaks, but suddenly there is no feedback from a hardware interface.
  Or it can help you to test your setup when the hardware is running without feedback, i.e., in open loop configuration.

mock_gpio_commands (optional; boolean; default: false)
  Creates fake command interfaces for faking GPIO states with an external command.
  Those interfaces are usually used by a :ref:`forward controller <forward_command_controller_userdoc>` to provide access from ROS-world.

mock_sensor_commands (optional; boolean; default: false)
  Creates fake command interfaces for faking sensor measurements with an external command.
  Those interfaces are usually used by a :ref:`forward controller <forward_command_controller_userdoc>` to provide access from ROS-world.

position_state_following_offset (optional; double; default: 0.0)
  Following offset added to the commanded values when mirrored to states. Only applied, if ``custom_interface_with_following_offset`` is false.

Per-Interface Parameters
########################

initial_value (optional; double)
  Initial value of certain state interface directly after startup. Example:

  .. code-block:: xml

     <state_interface name="position">
       <param name="initial_value">3.45</param>
     </state_interface>

  Note: This parameter is shared with the gazebo and gazebo classic plugins for
  joint interfaces. For Mock components it is also possible to set initial
  values for gpio or sensor state interfaces.
