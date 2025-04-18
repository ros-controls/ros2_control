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
   Different Update Rates <different_update_rates_userdoc.rst>
   Asynchronous Execution <asynchronous_components.rst>
   Semantic Components <semantic_components.rst>


Lifecycle of a Hardware Component
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Methods return values have type
``rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn`` with the following
meaning:

* ``CallbackReturn::SUCCESS`` method execution was successful.
* ``CallbackReturn::FAILURE`` method execution has failed and the lifecycle transition is unsuccessful.
* ``CallbackReturn::ERROR`` critical error has happened that should be managed in
  ``on_error`` method.

The hardware transitions to the following state after each method:

* **UNCONFIGURED** (``on_init``, ``on_cleanup``):

  Hardware is only initialized, but communication is not started and no interfaces are imported into ``ResourceManager``.

* **INACTIVE** (``on_configure``, ``on_deactivate``):

  Communication with the hardware is established and hardware component is configured.
  States can be read and command interfaces (System and Actuator only) are available.

  As of now, it is left to the hardware component implementation to continue using the command received from the ``CommandInterfaces`` or to skip them completely.

  .. note::

    We plan to implement safety-critical interfaces, see this `PR in the roadmap <https://github.com/ros-controls/roadmap/pull/51/files>`__. But currently, all command interfaces are available and will be written, see this `issue <https://github.com/ros-controls/ros2_control/issues/931>`__ describing the situation.

* **FINALIZED** (``on_shutdown``):

  Hardware interface is ready for unloading/destruction.
  Allocated memory is cleaned up.

* **ACTIVE** (``on_activate``):

  States can be read.

  System and Actuator only:

    Power circuits of hardware are active and hardware can be moved, e.g., brakes are disengaged.
    Command interfaces are available and the commands should be sent to the hardware


Handling of errors that happen during read() and write() calls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If ``hardware_interface::return_type::ERROR`` is returned from ``read()`` or ``write()`` methods of a hardware_interface class, ``on_error(previous_state)`` method will be called to handle any error that happened.

Error handling follows the `node lifecycle <https://design.ros2.org/articles/node_lifecycle.html>`_.
If successful ``CallbackReturn::SUCCESS`` is returned and hardware is again in ``UNCONFIGURED``  state, if any ``ERROR`` or ``FAILURE`` happens the hardware ends in ``FINALIZED`` state and can not be recovered.
The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.
