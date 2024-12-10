:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Galactic to Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Galactic (previous) and Humble (current) releases. Bugfixes are not included in this list.

.. note::

  This list was created in July 2024, earlier changes may not be included.

controller_interface
********************

* The new ``PoseSensor`` semantic component provides a standard interface for hardware providing cartesian poses (`#1775 <https://github.com/ros-controls/ros2_control/pull/1775>`_)

controller_manager
******************

* ``ros2_control_node`` can now handle the sim time used by different simulators, when ``use_sim_time`` is set to true (`#1810 <https://github.com/ros-controls/ros2_control/pull/1810>`_).
* The ``ros2_control_node`` node now accepts the ``thread_priority`` parameter to set the scheduler priority of the controller_manager's RT thread (`#1820 <https://github.com/ros-controls/ros2_control/pull/1820>`_).
* Added support for the wildcard entries for the controller configuration files (`#1724 <https://github.com/ros-controls/ros2_control/pull/1724>`_).
* The ``ros2_control_node`` node has a new ``lock_memory`` parameter to lock memory at startup to physical RAM in order to avoid page faults (`#1822 <https://github.com/ros-controls/ros2_control/pull/1822>`_).
* The ``ros2_control_node`` node has a new ``cpu_affinity`` parameter to bind the process to a specific CPU core. By default, this is not enabled. (`#1852 <https://github.com/ros-controls/ros2_control/pull/1852>`_).
* ``--switch-timeout`` was added as parameter to the helper scripts ``spawner.py`` and ``unspawner.py``. Useful if controllers cannot be switched immediately, e.g., paused simulations at startup (`#1790 <https://github.com/ros-controls/ros2_control/pull/1790>`_).
* The spawner now supports parsing multiple ``-p`` or ``--param-file`` arguments, this should help in loading multiple parameter files for a controller or for multiple controllers (`#1805 <https://github.com/ros-controls/ros2_control/pull/1805>`_).
* The ``--service-call-timeout`` was added as parameter to the helper scripts ``spawner.py``. Useful when the CPU load is high at startup and the service call does not return immediately (`#1808 <https://github.com/ros-controls/ros2_control/pull/1808>`_).
