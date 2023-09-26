:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/hardware_components_userdoc.rst

.. _hardware_components_userdoc:

Hardware Components
-------------------
Hardware components represent abstraction of physical hardware in ros2_control framework.
There are three types of hardware Actuator, Sensor and System.
For details on each type check :ref:`overview_hardware_components` description.

Guidelines and Best Practices
*****************************

.. toctree::
   :titlesonly:

   Hardware Interface Types <hardware_interface_types_userdoc.rst>
   Writing a Hardware Component <writing_new_hardware_component.rst>


Handling of errors that happen during read() and write() calls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If ``hardware_interface::return_type::ERROR`` is returned from ``read()`` or ``write()`` methods of a hardware_interface class, ``on_error(previous_state)`` method will be called to handle any error that happened.

Error handling follows the `node lifecycle <https://design.ros2.org/articles/node_lifecycle.html>`_.
If successful ``CallbackReturn::SUCCESS`` is returned and hardware is again in ``UNCONFIGURED``  state, if any ``ERROR`` or ``FAILURE`` happens the hardware ends in ``FINALIZED`` state and can not be recovered.
The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.

Migration from Foxy to newer versions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
