:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.


controller_interface
********************

controller_manager
******************

hardware_interface
******************

* The signature for the ``on_init`` method in all
  ``hardware_interface::*Interface`` classes has changed (`#2323
  <https://github.com/ros-controls/ros2_control/pull/2323>`_,
  `#2589 <https://github.com/ros-controls/ros2_control/pull/2589>`__) from

  .. code-block:: cpp

     CallbackReturn on_init(const hardware_interface::HardwareInfo& info)

  to

  .. code-block:: cpp

     CallbackReturn on_init(const HardwareComponentInterfaceParams& params)

  The ``HardwareInfo`` object can be accessed from the ``HardwareComponentInterfaceParams`` object using
  ``params.hardware_info``. See :ref:`writing_new_hardware_component` for advanced usage of the
  ``HardwareComponentInterfaceParams`` object.

* The signature for the ``init()`` method in all
  ``hardware_interface::*Interface`` classes has changed (`#2344
  <https://github.com/ros-controls/ros2_control/pull/2344>`_,
  `#2589 <https://github.com/ros-controls/ros2_control/pull/2589>`__) from


  .. code-block:: cpp

     CallbackReturn init(const HardwareInfo & hardware_info, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)

  to

  .. code-block:: cpp

     CallbackReturn init(const hardware_interface::HardwareComponentParams & params)


* The ``initialize`` methods of all hardware components (such as ``Actuator``, ``Sensor``, etc.)
  have been changed from passing a ``const HardwareInfo &`` to passing a ``const
  HardwareComponentParams &`` (`#2323 <https://github.com/ros-controls/ros2_control/pull/2323>`_,
  `#2589 <https://github.com/ros-controls/ros2_control/pull/2589>`__).
