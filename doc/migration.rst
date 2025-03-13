:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Galactic to Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
<<<<<<< HEAD
This list summarizes important changes between Galactic (previous) and Humble (current) releases, where changes to user code might be necessary.
=======
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
* The support for the ``description`` parameter for loading the URDF was removed (`#1358 <https://github.com/ros-controls/ros2_control/pull/1358>`_). Use ``robot_description`` topic instead, e.g., you can use the `robot_state_publisher <https://index.ros.org/p/robot_state_publisher/#{DISTRO}>`_. For an example, see `this PR <https://github.com/ros-controls/ros2_control_demos/pull/456>`_ where the change was applied to the demo repository.

hardware_interface
******************
* ``test_components`` was moved to its own package. Update the dependencies if you are using them. (`#1325 <https://github.com/ros-controls/ros2_control/pull/1325>`_)
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.
* A new ``get_optional`` that returns a ``std::optional`` was added to the ``CommandInterface`` and ``StateInterface``. This can be used to check if the value is available or not. (`#1976 <https://github.com/ros-controls/ros2_control/pull/1976>`_ and `#2061 <https://github.com/ros-controls/ros2_control/pull/2061>`_)

Adaption of Command-/StateInterfaces
***************************************

* The handles for ``Command-/StateInterfaces`` have new set/get methods to access the values.
* ``Command-/StateInterfaces`` are now created and exported automatically by the framework via the ``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` methods based on the interfaces defined in the ``ros2_control`` XML-tag, which get parsed and the ``InterfaceDescription`` is created accordingly (check the `hardware_info.hpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/hardware_info.hpp>`__). The memory is now allocated in the handle itself.

Access to Command-/StateInterfaces
----------------------------------

Earlier code will issue compile-time warnings like:

.. code::

  warning: ‘double hardware_interface::Handle::get_value() const’ is deprecated: Use std::optional<T> get_optional() or bool get_value(double & value) instead to retrieve the value. [-Wdeprecated-declarations]
  warning: ignoring return value of ‘bool hardware_interface::Handle::set_value(const T&) [with T = double]’ [-Wunused-result]

The old methods are deprecated and will be removed in the future. The new methods are:

  * ``std::optional<T> get_optional()`` or ``bool get_value(T & value)`` for getting the value.
  * ``bool set_value(const T & value)`` for setting the value.

The return value ``bool`` or ``std::optional<T>`` with ``get_value`` can be used to check if the value is available or not. Similarly, the ``set_value`` method returns a ``bool`` to check if the value was set or not.
The ``get_value`` method will return an empty ``std::nullopt`` or ``false`` if the value is not available. The ``set_value`` method will return ``false`` if the value was not set.
>>>>>>> 3cb590c (Fix links in docs (#2070))

.. note::

  This list was created in July 2024, earlier changes are not included.
