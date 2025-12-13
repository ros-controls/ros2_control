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

* The ``get_value`` of LoanedStateInterface and LoanedCommandInterface is now accessed using ``get_optional`` method. The value will be returned as an ``std::optional<T>``. (`#2061 <https://github.com/ros-controls/ros2_control/pull/2061>`_).

  This change was made to better handle cases where the interface value may not be accessible due to a concurrent access from other threads in the system.

* The ``double get_value()`` of standard StateInterface and CommandInterface is now accessed using  ``get_optional`` or ``bool get_value(T & value, bool wait_for_lock)`` method. The value will be returned as an ``std::optional<T>`` when using ``get_optional`` (`#2831 <https://github.com/ros-controls/ros2_control/pull/2831>`_).

  Likewise, the ``set_value`` method has been updated to ``bool set_value(const T & value, bool wait_for_lock)`` and return value is to indicate success or failure of the operation (`#2831 <https://github.com/ros-controls/ros2_control/pull/2831>`_).

  You can use the return values of these methods to handle cases where the interface value may not be accessible due to a concurrent access from other threads in the system. You can set the ``wait_for_lock`` parameter to ``true`` to block until the lock is acquired, however, this is not real-time safe and should be used with caution in real-time contexts.
