.. _controller_manager_userdoc:

Controller Manager
==================
Controller Manager is the main component in the ros2_control framework.
It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world.

Determinism
-----------

For best performance when controlling hardware you want the controller manager to have as little jitter as possible in the main control loop.
The normal linux kernel is optimized for computational throughput and therefore is not well suited for hardware control.
The main thread of Controller Manager attempts to configure ``SCHED_FIFO`` with a priority of ``50``.
To enable this functionality install a RT kernel and run the Controller Manager with permissions to make syscalls to set its thread priorities.
The two easiest options for this are using the [Real-time Ubuntu 22.04 LTS Beta](https://ubuntu.com/blog/real-time-ubuntu-released) or [linux-image-rt-amd64](https://packages.debian.org/bullseye/linux-image-rt-amd64) on Debian Bullseye.

Helper scripts
--------------
There are two scripts to interact with controller manager from launch files:

  1. ``spawner`` - loads, configures and start a controller on startup.
  2. ``unspawner`` - stops and unloads a controller.


``spawner``
^^^^^^^^^^^^^^

.. code-block:: console

    $ ros2 run controller_manager spawner -h
    usage: spawner [-h] [-c CONTROLLER_MANAGER] [-p PARAM_FILE] [--load-only] [--stopped] [-t CONTROLLER_TYPE] [-u]
                      [--controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT]
                      controller_name

    positional arguments:
      controller_name       Name of the controller

    optional arguments:
      -h, --help            show this help message and exit
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      -p PARAM_FILE, --param-file PARAM_FILE
                            Controller param file to be loaded into controller node before configure
      --load-only           Only load the controller and leave unconfigured.
      --stopped             Load and configure the controller, however do not start them
      -t CONTROLLER_TYPE, --controller-type CONTROLLER_TYPE
                            If not provided it should exist in the controller manager namespace
      -u, --unload-on-kill  Wait until this application is interrupted and unload controller
      --controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT
                            Time to wait for the controller manager


``unspawner``
^^^^^^^^^^^^^^^^

.. code-block:: console

    $ ros2 run controller_manager unspawner -h
    usage: unspawner [-h] [-c CONTROLLER_MANAGER] controller_name

    positional arguments:
      controller_name       Name of the controller

    optional arguments:
      -h, --help            show this help message and exit
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
