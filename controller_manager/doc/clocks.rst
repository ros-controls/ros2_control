:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_manager/doc/clocks.rst

.. _clocks:

Different Clocks used by Controller Manager
===========================================

The controller manager internally uses the following two different clocks for a non-simulation setup:

- ``RCL_ROS_TIME``: This clock is used mostly in the non-realtime loops.
- ``RCL_STEADY_TIME``: This clock is used mostly in the realtime loops for the ``read``, ``update``, and ``write`` loops. However, when the controller manager is used in a simulation environment, the ``RCL_ROS_TIME`` clock is used for triggering the ``read``, ``update``, and ``write`` loops.

The ``time`` argument in the ``read`` and ``write`` methods of the hardware components is of type ``RCL_STEADY_TIME``, as most of the hardware expects the time to be monotonic and not affected by the system time changes. However, the ``time`` argument in the ``update`` method of the controller is of type ``RCL_ROS_TIME`` as the controller is the one that interacts with other nodes or topics to receive the commands or publish the state. This ``time`` argument can be used by the controllers to validate the received commands or to publish the state at the correct timestamp.
The ``period`` argument in the ``read``, ``update`` and ``write`` methods is calculated using the trigger clock of type ``RCL_STEADY_TIME`` so it is always monotonic.

The reason behind using different clocks is to avoid the issues related to the affect of system time changes in the realtime loops. The ``ros2_control_node`` now also detects the overruns caused by the system time changes and longer execution times of the controllers and hardware components. The controller manager will print a warning message if the controller or hardware component misses the update cycle due to the system time changes or longer execution times.
