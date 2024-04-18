Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

component parser
*****************
Changes from `(PR #1256) <https://github.com/ros-controls/ros2_control/pull/1256>`__

* All ``joints`` defined in the ``<ros2_control>``-tag have to be present in the URDF received :ref:`by the controller manager <doc/ros2_control/controller_manager/doc/userdoc:subscribers>`, otherwise a ``std::runtime_error`` is thrown. This is to ensure that the URDF and the ``<ros2_control>``-tag are consistent. E.g., for configuration ports use ``gpio`` tags instead.
* The syntax for mimic joints is changed to the `official URDF specification <https://wiki.ros.org/urdf/XML/joint>`__. The parameters within the ``ros2_control`` tag are not supported any more. Instead of

  .. code-block:: xml

    <ros2_control name="GazeboSystem" type="system">
      <joint name="right_finger_joint">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.15</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      <joint name="left_finger_joint">
        <param name="mimic">right_finger_joint</param>
        <param name="multiplier">1</param>
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </ros2_control>

  define your mimic joints directly in the joint definitions:

  .. code-block:: xml

    <joint name="right_finger_joint" type="prismatic">
      <axis xyz="0 1 0"/>
      <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
      <parent link="base"/>
      <child link="finger_right"/>
      <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
    </joint>
    <joint name="left_finger_joint" type="prismatic">
      <mimic joint="right_finger_joint" multiplier="1" offset="0"/>
      <axis xyz="0 1 0"/>
      <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
      <parent link="base"/>
      <child link="finger_left"/>
      <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
    </joint>

Adaption of Command-/StateInterfaces
***************************************
Changes from `(PR #1240) <https://github.com/ros-controls/ros2_control/pull/1240>`__

* ``Command-/StateInterfaces`` are now created and exported automatically by the framework via the ``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` methods based on the interfaces defined in the ``ros2_control`` XML-tag, which gets parsed and the ``InterfaceDescription`` is created accordingly (check the `hardware_info.hpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/hardware_info.hpp>`__).
* The memory for storing the value of a ``Command-/StateInterfaces`` is no longer allocated in the hardware but instead in the ``Command-/StateInterfaces`` itself.
* To access the automatically created ``Command-/StateInterfaces`` we provide the ``std::unordered_map<std::string, InterfaceDescription>``, where the string is the fully qualified name of the interface and the ``InterfaceDescription`` is the configuration of the interface. The ``std::unordered_map<>`` are divided into ``type_state_interfaces_`` and ``type_command_interfaces_`` where type can be: ``joint``, ``sensor``, ``gpio`` and ``unlisted``. E.g. the ``CommandInterfaces`` for all joints can be found in the  ``joint_command_interfaces_`` map. The ``unlisted`` includes all interfaces not listed in the ``ros2_control`` XML-tag but were created by overriding the ``export_command_interfaces_2()`` or ``export_state_interfaces_2()`` function by creating some custom ``Command-/StateInterfaces``.

Migration of Command-/StateInterfaces
-------------------------------------
To adapt to the new way of creating and exporting ``Command-/StateInterfaces`` follow those steps:

1. Delete the ``std::vector<hardware_interface::CommandInterface> export_command_interfaces() override`` and ``std::vector<hardware_interface::StateInterface> export_state_interfaces() override``.
2. Delete allocated memory for any ``Command-/StateInterfaces``, e.g.:

  * If you have a ``std::vector<double> hw_commands_;`` for joints' ``CommandInterfaces`` delete the definition and any usage/appearance.
  * Wherever you iterated over a state/command or accessed commands like this:

.. code-block:: c++

    // states
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = 0;
      auto some_state = hw_states_[i];
    }

    // commands
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      hw_commands_[i] = 0;
      auto some_command = hw_commands_[i];
    }

    // specific state/command
    hw_commands_[x] = hw_states_[y];

replace it with

.. code-block:: c++

  // states replace with this
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
    auto some_state = get_state(name);
  }

  //commands replace with this
  for (const auto & [name, descr] : joint_commands_interfaces_)
  {
    set_command(name, 0.0);
    auto some_command = get_command(name);
  }

  // replace specific state/command, for this you need to store the names which are std::strings
  // somewhere or know them. However be careful since the names are fully qualified names which
  // means that the prefix is included for the name: E.g.: prefix/joint_1/velocity
  set_command(name_of_command_interface_x, get_state(name_of_state_interface_y));

Migration of unlisted Command-/StateInterfaces not defined in ``ros2_control`` XML-tag
--------------------------------------------------------------------------------------
If you want some unlisted ``Command-/StateInterfaces`` not included in the ``ros2_control`` XML-tag you can follow those steps:

1. Override the ``virtual std::vector<hardware_interface::InterfaceDescription> export_command_interfaces_2()`` or ``virtual std::vector<hardware_interface::InterfaceDescription> export_state_interfaces_2()``
2. Create the InterfaceDescription for each of the interfaces you want to create in the override ``export_command_interfaces_2()`` or ``export_state_interfaces_2()`` function, add it to a vector and return the vector:

  .. code-block:: c++

    std::vector<hardware_interface::InterfaceDescription> my_unlisted_interfaces;

    InterfaceInfo unlisted_interface;
    unlisted_interface.name = "some_unlisted_interface";
    unlisted_interface.min = "-5.0";
    unlisted_interface.data_type = "double";
    my_unlisted_interfaces.push_back(InterfaceDescription(info_.name, unlisted_interface));

    return my_unlisted_interfaces;

3. The unlisted interface will then be stored in either the ``unlisted_command_interfaces_`` or ``unlisted_state_interfaces_`` map depending in which function they are created.
4. You can access it like any other interface with the ``get_state(name)``, ``set_state(name, value)``, ``get_command(name)`` or ``set_command(name, value)``. E.g. ``get_state("some_unlisted_interface")``.

Custom exportation of Command-/StateInterfaces
----------------------------------------------
In case the default implementation (``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` ) for exporting the ``Command-/StateInterfaces`` is not enough you can override them. You should however consider the following things:

* If you want to have unlisted interfaces available you need to call the ``export_command_interfaces_2()`` or ``export_state_interfaces_2()`` and add them to the ``unlisted_command_interfaces_`` or ``unlisted_state_interfaces_``.
* Don't forget to store the created ``Command-/StateInterfaces`` internally as you only return ``shared_ptrs`` and the resource manager will not provide access to the created ``Command-/StateInterfaces`` for the hardware. So you must take care of storing them yourself.
* Names must be unique!
