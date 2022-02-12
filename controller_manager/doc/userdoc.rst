.. _controller_manager_userdoc:

Controller Manager
==================
Controller Manager is the main component in the ros2_control framework.
It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world.


Parameters
-----------

activate_components_on_start (optional; list<string>; default: empty)
  Define which hardware components should be activated when controller manager is started.
  The names of the components are defined as attribute of ``<ros2_control>``-tag in ``robot_description``.
  All other components will stay ``UNCONFIGURED``.
  If this and ``configure_components_on_start`` are empty, all available components will be activated.
  If this or ``configure_components_on_start`` are not empty, any component not in either list will be in unconfigured state.


configure_components_on_start (optional; list<string>; default: empty)
  Define which hardware components should be configured when controller manager is started.
  The names of the components are defined as attribute of ``<ros2_control>``-tag in ``robot_description``.
  All other components will stay ``UNCONFIGURED``.
  If this and ``activate_components_on_start`` are empty, all available components will be activated.
  If this or ``activate_components_on_start`` are not empty, any component not in either list will be in unconfigured state.


robot_description (mandatory; string)
  String with the URDF string as robot description.
  This is usually result of the parsed description files by ``xacro`` command.

update_rate (mandatory; double)
  The frequency of controller manager's real-time update loop.
  This loop reads states from hardware, updates controller and writes commands to hardware.


<controller_name>.type
  Name of a plugin exported using ``pluginlib`` for a controller.
  This is a class from which controller's instance with name "``controller_name``" is created.


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
