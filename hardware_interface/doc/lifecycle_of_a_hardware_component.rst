:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/lifecycle_of_a_hardware_component.rst

Lifecycle of a Hardware Component
---------------------------------
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
  States can be read, but command interfaces (System and Actuator only) are not available.

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
