:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes/Galactic.rst

Foxy to Galactic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

hardware_interface
**************************************
Between Foxy and Galactic we made substantial changes to the interface of hardware components to enable management of their lifecycle.
The following list shows a set of quick changes to port existing hardware components to Galactic:

1. Rename ``configure`` to ``on_init`` and change return type to ``CallbackReturn``

2. If using BaseInterface as base class then you should remove it. Specifically, change:

  .. code-block:: c++

    hardware_interface::BaseInterface<hardware_interface::[Actuator|Sensor|System]Interface>

  to

  .. code-block:: c++

    hardware_interface::[Actuator|Sensor|System]Interface

3. Remove include of headers ``base_interface.hpp`` and ``hardware_interface_status_values.hpp``

4. Add include of header ``rclcpp_lifecycle/state.hpp`` although this may not be strictly necessary

5. replace first three lines in ``on_init`` to

  .. code-block:: c++

    if (hardware_interface::[Actuator|Sensor|System]Interface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }


6. Change last return of ``on_init`` to ``return CallbackReturn::SUCCESS;``

7. Remove all lines with ``status_ = ...`` or ``status::...``

8. Rename ``start()`` to ``on_activate(const rclcpp_lifecycle::State & previous_state)`` and
   ``stop()`` to ``on_deactivate(const rclcpp_lifecycle::State & previous_state)``

9. Change return type of ``on_activate`` and ``on_deactivate`` to ``CallbackReturn``

10. Change last return of ``on_activate`` and ``on_deactivate`` to ``return CallbackReturn::SUCCESS;``

11. If you have any ``return_type::ERROR`` in ``on_init``, ``on_activate``, or ``in_deactivate`` change to ``CallbackReturn::ERROR``

12. If you get link errors with undefined refernences to symbols in ``rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface``, then add
    ``rclcpp_lifecyle`` package dependency to ``CMakeLists.txt`` and ``package.xml``
