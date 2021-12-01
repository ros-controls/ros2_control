.. _hardware_components_userdoc:

Hardware Components
-------------------
Hardware components represent abstraction of physical hardware in ros2_control framework.
There are three types of hardware Actuator, Sensor and System.
For details on each type check `Hardware Components description <https://ros-controls.github.io/control.ros.org/getting_started.html#hardware-components>`_.


Lifecycle of Hardware Components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Definitions and Nomenclature
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

Hardware interfaces use the lifecycle state machine `defined for ROS2 nodes <https://design.ros2.org/articles/node_lifecycle.html>`_.
There is only one addition to the state machine, that is the initialization method providing hardware configuration from URDF file as argument.

Hardware Interface
  User impl....

Hardware Components
  Wrapper and abstraction of hardware interface to manage life cycle and access to methods of hardware interface from Resource Manager.

Resource Manager
  Class responsible for the management of hardware components in the ros2_control framework.

"movement" command interfaces
  Interfaces responsible for robot to move, i.e., influence its dynamic behavior.
  The interfaces are defined in `hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_. (TODO: add link to doxygen)

"non-movement" command interfaces
  All other interfaces that are not "movement" command interfaces (TODO: add link to def.)


Initialization
,,,,,,,,,,,,,,,
Immediately after a plugin in loaded and the object created with the default constructor, the ``on_init`` method will be called providing hardware URDF configuration using the ``HardwareInfo`` structure.
In this stage you should initialize all memory you need and prepare storage for interfaces.
The resource manager will export of all interfaces after this and store them internally.


Configuration
,,,,,,,,,,,,,,
Precondition is hardware interface state having id: ``lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED``.
After configuration, the ``read`` and ``write`` methods will be called in the update loop.
This means all internal state and commands variables have to be initialized.
After a successful call to ``on_configure``, all state interfaces and "non-movement" command interfaces should be available to controllers.

NOTE: If using "non-movement" command interfaces to parametrize the robot in the ``lifecycle_msgs::msg::State::PRIMARY_STATE_CONFIGURED`` state make sure to take care about current state in the ``write`` method of your Hardware Interface implementation.


Handling of errors that happen during read() and write() calls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If ``hardware_interface::return_type::ERROR`` is returned from ``read()`` or ``write()`` methods of a hardware interface, ``on_error(previous_state)`` method will be called to handle any error that happened.

Error handling follows the `node lifecycle <https://design.ros2.org/articles/node_lifecycle.html>`_.
If successful ``CallbackReturn::SUCCESS`` is returned and hardware is again in ``UNCONFIGURED``  state, if any ``ERROR`` or ``FAILURE`` happens the hardware ends in ``FINALIZED`` state and can not be recovered.
The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.

Migration from Foxy to Galactic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Between Foxy and Galactic we made substantial changes to the interface of hardware components to enable management of their lifecycle.
The following list shows a set of quick changes to port existing hardware components to Galactic:

1. Rename ``configure`` to ``on_init`` and change return type to ``CallbackReturn``

2. If using BaseInterface as base class then you should remove it. Specifically, change:

hardware_interface::BaseInterface<hardware_interface::[Actuator|Sensor|System]Interface> to hardware_interface::[Actuator|Sensor|System]Interface

3. Remove include of headers ``base_interface.hpp`` and ``hardware_interface_status_values.hpp``

4. Add include of header ``rclcpp_lifecycle/state.hpp`` although this may not be strictly necessary

5. replace first three lines in ``on_init`` to:

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
