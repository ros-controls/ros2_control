:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration/Jazzy.rst

Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
controller_interface
********************
* The changes from `(PR #1694) <https://github.com/ros-controls/ros2_control/pull/1694>`__ will affect how the controllers will be loading the parameters. Defining parameters in a single yaml file and loading it to the controller_manager node alone will no longer work.
  In order to load the parameters to the controllers properly, it is needed to use ``--param-file`` option from the spawner. This is because the controllers will now set ``use_global_arguments`` from `NodeOptions <https://docs.ros.org/en/rolling/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html#_CPPv4N6rclcpp11NodeOptions20use_global_argumentsEb>`__ to false, to avoid getting influenced by global arguments.
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.

controller_manager
******************
* Rename ``class_type`` to ``plugin_name`` (`#780 <https://github.com/ros-controls/ros2_control/pull/780>`_)
* CM now subscribes to ``robot_description`` topic instead of ``~/robot_description`` (`#1410 <https://github.com/ros-controls/ros2_control/pull/1410>`_). As a consequence, when using multiple controller managers, you have to remap the topic within the launch file, an example for a python launch file:

  .. code-block:: python

    remappings=[
                ('/robot_description', '/custom_1/robot_description'),
            ]

* Changes from `(PR #1256) <https://github.com/ros-controls/ros2_control/pull/1256>`__

  * All ``joints`` defined in the ``<ros2_control>``-tag have to be present in the URDF received :ref:`by the controller manager <doc/ros2_control/controller_manager/doc/userdoc:subscribers>`, otherwise the following error is shown:

      The published robot description file (URDF) seems not to be genuine. The following error was caught: <unknown_joint> not found in URDF.

    This is to ensure that the URDF and the ``<ros2_control>``-tag are consistent. E.g., for configuration ports use ``gpio`` interface types instead.

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
* The support for the ``description`` parameter for loading the URDF was removed (`#1358 <https://github.com/ros-controls/ros2_control/pull/1358>`_). Use ``robot_description`` topic instead, e.g., you can use the `robot_state_publisher <https://index.ros.org/p/robot_state_publisher/github-ros-robot_state_publisher/#{DISTRO}>`_. For an example, see `this PR <https://github.com/ros-controls/ros2_control_demos/pull/456>`_ where the change was applied to the demo repository.

hardware_interface
******************
* ``test_components`` was moved to its own package. Update the dependencies if you are using them. (`#1325 <https://github.com/ros-controls/ros2_control/pull/1325>`_)
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.

Adaption of Command-/StateInterfaces
***************************************

* ``Command-/StateInterfaces`` are now created and exported automatically by the framework via the ``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` methods based on the interfaces defined in the ``ros2_control`` XML-tag, which get parsed and the ``InterfaceDescription`` is created accordingly (check the `hardware_info.hpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/hardware_info.hpp>`__). The memory is now allocated in the handle itself.

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

  // replace specific state/command, for this you need to store the names which are strings
  // somewhere or know them. However be careful since the names are fully qualified names which
  // means that the prefix is included for the name: E.g.: prefix/joint_1/velocity
  set_command(name_of_command_interface_x, get_state(name_of_state_interface_y));

Migration of unlisted Command-/StateInterfaces not defined in ``ros2_control`` XML-tag
--------------------------------------------------------------------------------------
If you want some unlisted ``Command-/StateInterfaces`` not included in the ``ros2_control`` XML-tag you can follow those steps:

1. Override the ``virtual std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interfaces()`` or ``virtual std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interfaces()``
2. Create the InterfaceDescription for each of the interfaces you want to create in the override ``export_unlisted_command_interfaces()`` or ``export_unlisted_state_interfaces()`` function, add it to a vector and return the vector:

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

Custom export of Command-/StateInterfaces
----------------------------------------------
In case the default implementation (``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` ) for exporting the ``Command-/StateInterfaces`` is not enough you can override them. You should however consider the following things:

* If you want to have unlisted interfaces available you need to call the ``export_unlisted_command_interfaces()`` or ``export_unlisted_state_interfaces()`` and add them to the ``unlisted_command_interfaces_`` or ``unlisted_state_interfaces_``.
* Don't forget to store the created ``Command-/StateInterfaces`` internally as you only return ``std::shared_ptr`` and the resource manager will not provide access to the created ``Command-/StateInterface`` for the hardware. So you must take care of storing them yourself.
* Names must be unique!
