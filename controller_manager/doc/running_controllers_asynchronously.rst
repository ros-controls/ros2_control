:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_manager/doc/running_controllers_asynchronously.rst

.. _running_controllers_asynchronously:

Running Controllers Asynchronously
====================================

The ``ros2_control`` framework allows controllers to run asynchronously.
This is useful when a controller's ``update()`` method contains blocking calls or requires
This is useful when a controller's ``update()`` method contains blocking calls or requires more execution time that affects the controller manager's control loop period, which would
otherwise affect the periodicity of the control loop causing overruns.

For example, if the ``update_rate`` for the controller manager is 100Hz, the sum of the
execution times of all controllers' ``update()`` calls and hardware components ``read()``
and ``write()`` calls must be below 10ms. If one controller requires 15ms of execution
time, it cannot be executed synchronously without affecting the overall system update rate.
Running such a controller asynchronously allows it to execute on a dedicated thread without
blocking the main control loop.

.. note::
  The async update support is transparent to each controller implementation.
  Any existing controller can be run asynchronously just by setting
  the ``is_async`` parameter to ``true``.

Parameters
-----------

The following parameters can be set in the controller's configuration to run it
asynchronously:

* ``is_async``: (optional) If set to ``true``, the controller will run its ``update()``
  method asynchronously. Default is ``false``.
* ``update_rate``: (optional) The rate in Hz at which the controller's ``update()`` is
  triggered. If not set or set to the rate of the
  controller manager.

Additional thread parameters can be set under the ``async_parameters`` namespace:

* ``thread_priority``: (optional) The priority of the thread that runs the controller's
  ``update()``. The priority is an integer value between 0 and 99. If not set, the thread
  inherits the same priority as the controller manager thread.
* ``cpu_affinity``: (optional) A list of CPU core IDs to pin the async thread to.
  Default is an empty list, meaning the thread can run on any CPU core.

.. note::
  The thread priority is only used when the controller runs asynchronously.
  Asynchronous controller threads use the FIFO scheduling policy.

Examples
---------

The following example shows how to configure an asynchronous controller in a ROS 2
YAML parameters file. The controller manager runs at 100Hz and the async controller
is configured to run at 20Hz:

.. code-block:: yaml

    controller_manager:
      ros__parameters:
        update_rate: 100  # Hz

    example_async_controller:
      ros__parameters:
        type: example_controller/ExampleController
        is_async: true
        update_rate: 20  # Hz

This will result in the controller being triggered at 20Hz while the controller manager
runs at 100Hz.

Additional thread parameters can be configured using the ``async_parameters`` namespace:

.. code-block:: yaml

    example_async_controller:
      ros__parameters:
        type: example_controller/ExampleController
        is_async: true
        update_rate: 20  # Hz
        async_parameters:
          thread_priority: 50
          cpu_affinity: [2, 4]

The description of all parameters can be found in the `Common Controller Parameters
<https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html#common-controller-parameters>`_
section of the ``ros2_controllers`` documentation.

Scheduling Behavior
--------------------

From a design perspective, the controller manager functions as a scheduler that triggers
updates for asynchronous controllers during the control loop.

The ``ControllerInterfaceBase`` calls ``AsyncFunctionHandler`` to handle the actual
``update()`` callback of the controller. This is the same mechanism used by the resource
manager to support ``read()`` and ``write()`` operations for asynchronous hardware
components (see :ref:`asynchronous_components`). When a controller is configured to run
asynchronously, the controller interface creates an async handler during the controller's
configuration phase and binds it to the controller's ``update()`` method. The async
handler thread is created with either the same thread priority as the controller manager
or the priority specified by the ``thread_priority`` parameter.

When triggered by the controller manager, the async handler checks whether the previous
trigger has completed and then calls the ``update()`` method. If the previous ``update()``
is still running when the next trigger arrives, the previous result is reused and the
controller manager prints a warning:

.. code-block:: console

   [ros2_control_node-1] [WARN] [1741626670.311533972] [example_async_controller]: The controller missed xx update cycles out of yy total triggers.

If this warning appears frequently, the controller's ``update_rate`` should be reduced, as
the computation is taking longer than the configured period allows.

If the async controller's ``update()`` method throws an unhandled exception, the controller
manager handles it the same way as for synchronous controllers — by deactivating the
controller. An error message is printed similar to the following:

.. code-block:: console

  [ros2_control_node-1] [ERROR] [1741629098.352771957] [AsyncFunctionHandler]: AsyncFunctionHandler: Exception caught in the async callback thread!
  ...
  [ros2_control_node-1] [ERROR] [1741629098.352874151] [controller_manager]: Caught exception of type : St13runtime_error while updating controller
  [ros2_control_node-1] [ERROR] [1741629098.352940701] [controller_manager]: Deactivating controllers : [example_async_controller] as their update resulted in an error!

Monitoring and Tuning
----------------------

The ``controller_interface`` package provides a ``ControllerUpdateStats`` structure which
can be used to monitor the controller update rate and missed update cycles. This data is
published to the ``/diagnostics``  and also `/controller_manager/introspection_data/*` topic and can be used to fine-tune the controller's
``update_rate``.

See Also
---------

* :ref:`asynchronous_components` — Running hardware components asynchronously
* :ref:`different_update_rates_userdoc` — Running hardware components at different update rates
