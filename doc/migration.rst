:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.


controller_interface
********************

ChainableControllerInterface
----------------------------

* The ``on_export_state_interfaces()`` method has been removed and replaced by ``on_export_state_interfaces_list()`` (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_, `#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). The new method returns shared pointers instead of objects by value:

  .. code-block:: cpp

     // Old (removed)
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
       my_state_itf_ = std::make_shared<hardware_interface::StateInterface>(
         std::string(get_node()->get_name()) + "/my_state", "position");
       std::ignore = my_state_itf_->set_value(std::numeric_limits<double>::quiet_NaN());
       return {my_state_itf_};
     }

* The ``on_export_reference_interfaces()`` method has been removed and replaced by ``on_export_reference_interfaces_list()`` (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_, `#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). ``on_export_reference_interfaces_list()`` is pure virtual, so every chainable controller must implement it, even if it exports no reference interfaces. The new method returns shared pointers instead of objects by value:

  .. code-block:: cpp

     // Old (removed)
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
       my_ref_itf_ = std::make_shared<hardware_interface::CommandInterface>(
         std::string(get_node()->get_name()) + "/my_ref", "velocity");
       std::ignore = my_ref_itf_->set_value(std::numeric_limits<double>::quiet_NaN());
       return {my_ref_itf_};
     }

* The exported state interfaces are now returned as ``ConstSharedPtr`` from ``ChainableControllerInterface::export_state_interfaces()`` to ensure they are read-only for consumers (`#1767 <https://github.com/ros-controls/ros2_control/pull/1767>`_).

* The internal storage variables ``reference_interfaces_`` and ``state_interfaces_values_`` have been removed (`#2988 <https://github.com/ros-controls/ros2_control/pull/2988>`_, `#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). Values now live inside the exported interfaces themselves. Keep the shared pointers created in ``on_export_*_interfaces_list()`` as members, or use the ordered exported interface containers (``ordered_exported_state_interfaces_`` and ``ordered_exported_reference_interfaces_``), and access the values through ``get_optional()`` and ``set_value()`` in ``update_reference_from_subscribers()``, ``update_and_write_commands()`` and ``on_set_chained_mode()``:

  .. code-block:: cpp

     // Old implementation
     controller_interface::return_type MyController::update_and_write_commands(
       const rclcpp::Time &, const rclcpp::Duration &)
     {
       if (!std::isnan(reference_interfaces_[0]))
       {
         std::ignore = command_interfaces_[0].set_value(reference_interfaces_[0]);
       }
       my_state_value_ = state_interfaces_[0].get_optional().value_or(my_state_value_);
       return controller_interface::return_type::OK;
     }

     // New implementation
     // my_ref_itf_ and my_state_itf_ are stored in on_export_*_interfaces_list()
     controller_interface::return_type MyController::update_and_write_commands(
       const rclcpp::Time &, const rclcpp::Duration &)
     {
       const double reference =
         my_ref_itf_->get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
       if (!std::isnan(reference))
       {
         std::ignore = command_interfaces_[0].set_value(reference);
       }
       if (const auto state = state_interfaces_[0].get_optional(); state.has_value())
       {
         std::ignore = my_state_itf_->set_value(state.value());
       }
       return controller_interface::return_type::OK;
     }

  ``get_optional()`` returns ``std::nullopt`` and ``set_value()`` returns ``false`` if the interface lock could not be acquired, so handle these cases instead of assuming access always succeeds.

  Controllers exporting several interfaces can build them from a fixed list of names (`#3350 <https://github.com/ros-controls/ros2_control/issues/3350>`__) and keep the returned pointers indexed the same way the old ``reference_interfaces_`` vector was:

  .. code-block:: cpp

     static constexpr std::array<std::string_view, 3> reference_interface_names = {
       "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};

     std::vector<hardware_interface::CommandInterface::SharedPtr>
     MyController::on_export_reference_interfaces_list()
     {
       ref_itfs_.clear();
       for (const auto & name : reference_interface_names)
       {
         auto itf = std::make_shared<hardware_interface::CommandInterface>(
           get_node()->get_name(), std::string{name});
         std::ignore = itf->set_value(std::numeric_limits<double>::quiet_NaN());
         ref_itfs_.push_back(itf);
       }
       return ref_itfs_;
     }

  `ros2_controllers#2350 <https://github.com/ros-controls/ros2_controllers/pull/2350>`__ migrated the first-party chainable controllers and can be used as a reference.

  Migration checklist for a chainable controller:

  #. Mark the existing ``on_export_state_interfaces()`` and ``on_export_reference_interfaces()`` overrides with ``override``. If an old method is declared without ``override``, it still compiles but is never called, so its interfaces are silently not exported; adding ``override`` makes the compiler flag every method that has to change.
  #. Rename ``on_export_state_interfaces()`` to ``on_export_state_interfaces_list()`` and ``on_export_reference_interfaces()`` to ``on_export_reference_interfaces_list()``, changing the return types to ``std::vector<...::SharedPtr>``.
  #. Replace every ``emplace_back(prefix, name, &storage)`` with ``std::make_shared<...>(prefix, name)``, set the initial value with ``set_value()``, and store the pointer in a member. The prefix of every exported interface must begin with the controller's name (``get_node()->get_name()``), otherwise exporting throws a ``std::runtime_error``.
  #. Implement ``on_export_reference_interfaces_list()`` even if the controller has no reference interfaces; return an empty vector in that case.
  #. Remove the ``reference_interfaces_.resize(...)`` and ``state_interfaces_values_.resize(...)`` calls, usually found in ``on_configure()`` or the export methods.
  #. Replace every read of ``reference_interfaces_[i]`` with ``ref_itfs_[i]->get_optional()`` and every write with ``ref_itfs_[i]->set_value(...)``. Do the same for the member variables that backed the exported state interfaces. Check ``update_reference_from_subscribers()``, ``update_and_write_commands()``, ``on_set_chained_mode()``, ``on_activate()`` and ``on_deactivate()``, plus the controller's tests.
  #. Search the package for ``reference_interfaces_``, ``state_interfaces_values_``, ``on_export_state_interfaces()`` and ``on_export_reference_interfaces()`` in the controller sources; no matches should remain. Hardware components in the same package still use ``on_export_state_interfaces()``, so ignore matches there.

controller_manager
******************

* ``configure_controller`` now performs a single-step lifecycle transition and
  only accepts controllers in the ``unconfigured`` state (`#3196
  <https://github.com/ros-controls/ros2_control/pull/3196>`__). Previously an
  ``inactive`` controller was implicitly cleaned up and then reconfigured. To
  reconfigure an ``inactive`` controller, call ``cleanup_controller`` first and
  then ``configure_controller``.

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

* The ``export_state_interfaces()`` and ``export_command_interfaces()`` methods of ``HardwareComponentInterface`` have been removed (`#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). Hardware components can no longer export interfaces backed by their own member variables. Interfaces defined in the ``ros2_control`` tag are created and exported by the framework, and their values are accessed with ``set_state``/``get_state`` and ``set_command``/``get_command``:

  .. code-block:: cpp

     // Old implementation
     std::vector<hardware_interface::StateInterface> MyHardware::export_state_interfaces()
     {
       std::vector<hardware_interface::StateInterface> state_interfaces;
       state_interfaces.emplace_back(
         info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_);
       return state_interfaces;
     }

     hardware_interface::return_type MyHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
     {
       hw_position_ = read_position_from_hardware();
       return hardware_interface::return_type::OK;
     }

     // New implementation, no export method needed
     hardware_interface::return_type MyHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
     {
       set_state(info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION,
                 read_position_from_hardware());
       return hardware_interface::return_type::OK;
     }

  Interfaces not listed in the ``ros2_control`` tag are added by overriding ``export_unlisted_state_interface_descriptions()`` or ``export_unlisted_command_interface_descriptions()``. Override ``on_export_state_interfaces()`` or ``on_export_command_interfaces()`` only if full control over the exported interfaces is needed. See :ref:`writing_new_hardware_component` for details.

  Loops over the old storage vectors map to loops over the interface maps of the framework (``joint_state_interfaces_``, ``joint_command_interfaces_``, and likewise for ``sensor_``, ``gpio_`` and ``unlisted_``), keyed by the fully qualified interface name, e.g. ``prefix/joint_1/velocity``. The example below assumes all interfaces in the map are of type ``double``; ``set_state`` throws for interfaces of other data types:

  .. code-block:: cpp

     // Old implementation
     for (size_t i = 0; i < hw_states_.size(); ++i)
     {
       hw_states_[i] = 0.0;
     }
     hw_commands_[x] = hw_states_[y];

     // New implementation
     for (const auto & [name, descr] : joint_state_interfaces_)
     {
       set_state(name, 0.0);
     }
     set_command(name_of_command_interface_x, get_state(name_of_state_interface_y));

  Migration checklist for a hardware component:

  #. Mark the existing ``export_state_interfaces()`` and ``export_command_interfaces()`` overrides with ``override``. If an old method is declared without ``override``, it still compiles but is never called, so its interfaces are silently not exported; adding ``override`` makes the compiler flag every method that has to change.
  #. Delete the ``export_state_interfaces()`` and ``export_command_interfaces()`` overrides.
  #. Make sure every interface they exported is listed in the ``ros2_control`` tag of the URDF; move the rest to ``export_unlisted_state_interface_descriptions()`` or ``export_unlisted_command_interface_descriptions()``.
  #. In ``read()``, replace assignments to the member variables that backed the state interfaces (e.g. ``hw_positions_[i] = ...``) with ``set_state("<joint>/<interface>", value)``.
  #. In ``write()``, replace reads of the member variables that backed the command interfaces with ``get_command<double>("<joint>/<interface>")``. Do the same in ``on_activate()``, ``on_deactivate()`` and ``perform_command_mode_switch()``.
  #. Delete the now unused storage vectors (``hw_states_``, ``hw_commands_`` and similar) and their ``resize()`` calls in ``on_init()``.
  #. Search the package for ``export_state_interfaces`` and ``export_command_interfaces``, and for addresses of the old storage members (e.g. ``&hw_``); no matches should remain.

  This migration has been possible since Jazzy, see `the Jazzy migration guide <https://control.ros.org/jazzy/doc/ros2_control/doc/migration.html#migration-of-command-stateinterfaces>`__.

* The ``Handle(prefix_name, interface_name, double * value_ptr)`` constructor of ``StateInterface`` and ``CommandInterface`` has been removed (`#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). Handles now always own their value. Construct them from an ``InterfaceDescription`` or with ``Handle(prefix_name, interface_name, data_type, initial_value)``, where ``data_type`` and ``initial_value`` are strings defaulting to ``"double"`` and ``""`` (NaN). For a double interface ``Handle(prefix_name, interface_name)`` is enough.

* ``Handle::operator bool()`` has been removed (`#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). Use ``is_valid()`` instead.

* ``set_value<T>()`` on a handle now throws a ``std::runtime_error`` if ``T`` does not match the handle's data type, including ``double`` (`#3610 <https://github.com/ros-controls/ros2_control/pull/3610>`__). This error is raised at runtime, not at compile time: ``set_value(0)`` on a double interface throws because ``0`` is an ``int``; use ``set_value(0.0)``. The same applies to ``set_state`` and ``set_command`` of hardware components.
