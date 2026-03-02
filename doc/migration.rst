:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.


controller_interface
********************

* ``get_ordered_interfaces`` now throws if the size of the output vector does not match the size of the input names vector (`#2528 <https://github.com/ros-controls/ros2_control/pull/2528>`__).
* ``controller_interface::init()`` has been deprecated, use the ``init(const controller_interface::ControllerInterfaceParams & params)`` method instead. (`#2390 <https://github.com/ros-controls/ros2_control/pull/2390>`__).
  For example, the following code:

  .. code-block:: cpp

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "controller_name";
    params.robot_description = "";
    params.update_rate = 50;
    params.node_namespace = "";
    params.node_options = controller.define_custom_node_options();
    controller.init(params);

controller_manager
******************

* The spawner now supports two new arguments ``--switch-asap`` and ``--no-switch-asap`` to control the behaviour of the spawner when switching controllers to be in realtime loop (or) non-realtime loop.
  By default, it is set to ``--no-switch-asap`` because when activating multiple controllers at same time might affect the realtime loop performance (`#2452 <https://github.com/ros-controls/ros2_control/pull/2453>`_).
  If it is needed to switch controllers in realtime loop, then the argument ``--switch-asap`` need to be parsed to the spawner.

hardware_interface
******************

* The preferred signature for the ``on_init`` method in all
  ``hardware_interface::*Interface`` classes has changed (`#
  2323 <https://github.com/ros-controls/ros2_control/pull/2323>`_) from

  .. code-block:: cpp

     CallbackReturn on_init(const hardware_interface::HardwareInfo& info)

  to

  .. code-block:: cpp

     CallbackReturn on_init(const HardwareComponentInterfaceParams& params)

  The ``HardwareInfo`` object can be accessed from the ``HardwareComponentInterfaceParams`` object using
  ``params.hardware_info``. Hardware implementations implementing the ``on_init`` method should
  update their method signature accordingly as the deprecated signature will be removed in a
  future release.

  See :ref:`writing_new_hardware_component` for advanced usage of the
  ``HardwareComponentInterfaceParams`` object.

* The preferred signature for the ``init()`` method in all
  ``hardware_interface::*Interface`` classes has changed (`#
  2344 <https://github.com/ros-controls/ros2_control/pull/2344>`_) from


  .. code-block:: cpp

     CallbackReturn init(const HardwareInfo & hardware_info, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)

  to

  .. code-block:: cpp

     CallbackReturn init(const hardware_interface::HardwareComponentParams & params)


* The ``initialize`` methods of all hardware components (such as ``Actuator``, ``Sensor``, etc.)
  have been updated from passing a ``const HardwareInfo &`` to passing a ``const
  HardwareComponentParams &`` (`# 2323 <https://github.com/ros-controls/ros2_control/pull/2323>`_).
  The old signatures are deprecated and will be removed in a future release.

* The ``thread_priority`` variable in the ``HardwareInfo`` struct has been deprecated in favor of newly
  introduced ``async_params`` variable that has more options in the ``HardwareComponentParams`` struct.
  The deprecated ``thread_priority`` variable will be removed in a future release. (`# 2477 <https://github.com/ros-controls/ros2_control/pull/2477>`_).
