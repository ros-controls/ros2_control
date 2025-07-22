:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

hardware_interface
******************

* The preferred signature for the ``on_init`` method in all
  ``hardware_interface::*Interface`` classes has changed (`#
  2344 <https://github.com/ros-controls/ros2_control/pull/2344>`_) from

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
  HardwareComponentParams &`` (`# 2344 <https://github.com/ros-controls/ros2_control/pull/2344>`_).
  The old signatures are deprecated and will be removed in a future release.
