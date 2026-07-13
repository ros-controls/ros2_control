:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.


controller_interface
********************

ChainableControllerInterface
----------------------------

* The ``on_export_state_interfaces()`` method is deprecated and replaced by ``on_export_state_interfaces_list()`` (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_). The new method returns shared pointers instead of objects by value:

  .. code-block:: cpp

     // Old (deprecated)
     std::vector<hardware_interface::StateInterface> on_export_state_interfaces()

     // New
     std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list()

  Example migration:

  .. code-block:: cpp

     // Old implementation
     std::vector<hardware_interface::StateInterface>
     MyController::on_export_state_interfaces()
     {
       std::vector<hardware_interface::StateInterface> state_interfaces;
       state_interfaces.emplace_back(
         std::string(get_node()->get_name()) + "/my_state", "position", &my_state_value_);
       return state_interfaces;
     }

     // New implementation
     std::vector<hardware_interface::StateInterface::SharedPtr>
     MyController::on_export_state_interfaces_list()
     {
       std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces;
       auto state_interface = std::make_shared<hardware_interface::StateInterface>(
         std::string(get_node()->get_name()) + "/my_state", "position");
       state_interface->set_value(std::numeric_limits<double>::quiet_NaN());
       state_interfaces.push_back(state_interface);
       return state_interfaces;
     }

* The ``on_export_reference_interfaces()`` method is deprecated and replaced by ``on_export_reference_interfaces_list()`` (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_). The new method returns shared pointers instead of objects by value:

  .. code-block:: cpp

     // Old (deprecated)
     std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces()

     // New
     std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()

  Example migration:

  .. code-block:: cpp

     // Old implementation
     std::vector<hardware_interface::CommandInterface>
     MyController::on_export_reference_interfaces()
     {
       reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
       std::vector<hardware_interface::CommandInterface> reference_interfaces;
       reference_interfaces.emplace_back(
         std::string(get_node()->get_name()) + "/my_ref", "velocity", &reference_interfaces_[0]);
       return reference_interfaces;
     }

     // New implementation
     std::vector<hardware_interface::CommandInterface::SharedPtr>
     MyController::on_export_reference_interfaces_list()
     {
       std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;
       auto cmd_interface = std::make_shared<hardware_interface::CommandInterface>(
         std::string(get_node()->get_name()) + "/my_ref", "velocity");
       cmd_interface->set_value(std::numeric_limits<double>::quiet_NaN());
       reference_interfaces.push_back(cmd_interface);
       return reference_interfaces;
     }

* The exported state interfaces are now returned as ``ConstSharedPtr`` from ``export_state_interfaces()`` to ensure they are read-only for consumers (`#1767 <https://github.com/ros-controls/ros2_control/pull/1767>`_).

* The internal storage variables will be removed in upcoming releases. Controllers should now use the ordered exported interface containers (``ordered_exported_state_interfaces_`` and ``ordered_exported_reference_interfaces_``) which store shared pointers instead of raw values (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_).

* The controller manager's ros arguments are no longer forwarded to the controllers via NodeOptions. (`#3016 <https://github.com/ros-controls/ros2_control/pull/3016>`__)
  So, any remapping done at the controller manager level will not be visible to the controllers anymore.
  It is recommended to use the ``--controller-ros-args`` option of the spawner to pass ros arguments to controllers.

  .. code-block:: python

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("/diffbot_base_controller/cmd_vel", "/cmd_vel")],
        output="both",
    )

  to

  .. code-block:: python

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--controller-ros-args",
            "--remap",
            "/diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

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
