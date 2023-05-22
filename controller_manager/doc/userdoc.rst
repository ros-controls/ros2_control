:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_manager/doc/userdoc.rst

.. _controller_manager_userdoc:

Controller Manager
==================
Controller Manager is the main component in the ros2_control framework.
It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world.

Determinism
-----------

For best performance when controlling hardware you want the controller manager to have as little jitter as possible in the main control loop.
The normal linux kernel is optimized for computational throughput and therefore is not well suited for hardware control.
The two easiest kernel options are the `Real-time Ubuntu 22.04 LTS Beta <https://ubuntu.com/blog/real-time-ubuntu-released>`_ or `linux-image-rt-amd64 <https://packages.debian.org/bullseye/linux-image-rt-amd64>`_ on Debian Bullseye.

If you have a realtime kernel installed, the main thread of Controller Manager attempts to configure ``SCHED_FIFO`` with a priority of ``50``.
By default, the user does not have permission to set such a high priority.
To give the user such permissions, add a group named realtime and add the user controlling your robot to this group:

.. code-block:: console

    $ sudo addgroup realtime
    $ sudo usermod -a -G realtime $(whoami)

Afterwards, add the following limits to the realtime group in ``/etc/security/limits.conf``:

.. code-block:: console

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

The limits will be applied after you log out and in again.

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

update_rate (mandatory; integer)
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
    usage: spawner [-h] [-c CONTROLLER_MANAGER] [-p PARAM_FILE] [-n NAMESPACE] [--load-only] [--inactive] [-t CONTROLLER_TYPE] [-u]
                      [--controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT]
                      controller_name

    positional arguments:
      controller_name       Name of the controller

    options:
      -h, --help            show this help message and exit
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      -p PARAM_FILE, --param-file PARAM_FILE
                            Controller param file to be loaded into controller node before configure
      -n NAMESPACE, --namespace NAMESPACE
                            Namespace for the controller
      --load-only           Only load the controller and leave unconfigured.
      --inactive            Load and configure the controller, however do not activate them
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

Using the Controller Manager in a Process
-----------------------------------------

The ``ControllerManager`` may also be instantiated in a process as a class, but proper care must be taken when doing so.
The reason for this is because the ``ControllerManager`` class inherits from ``rclcpp::Node``.

If there is more than one Node in the process, global node name remap rules can forcibly change the ``ControllerManager's`` node name as well, leading to duplicate node names.
This occurs whether the Nodes are siblings or exist in a hierarchy.

.. image:: images/global_general_remap.png

The workaround for this is to specify another node name remap rule in the ``NodeOptions`` passed to the ``ControllerManager`` node (causing it to ignore the global rule), or ensure that any remap rules are targeted to specific nodes.

.. image:: images/global_specific_remap.png

..
  TODO: (methylDragon) Update the proposed solution when https://github.com/ros2/ros2/issues/1377 is resolved

.. code-block:: cpp

    auto options = controller_manager::get_cm_node_options();
      options.arguments({
        "--ros-args",
        "--remap", "_target_node_name:__node:=dst_node_name",
        "--log-level", "info"});

      auto cm = std::make_shared<controller_manager::ControllerManager>(
        executor, "_target_node_name", "some_optional_namespace", options);

Concepts
-----------

Restarting all controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The simplest way to restart all controllers is by using ``switch_controllers`` services or CLI and adding all controllers to ``start`` and ``stop`` lists.
Note that not all controllers have to be restarted, e.g., broadcasters.

Restarting hardware
^^^^^^^^^^^^^^^^^^^^^

If hardware gets restarted then you should go through its lifecycle again.
This can be simply achieved by returning ``ERROR`` from ``write`` and ``read`` methods of interface implementation.
**NOT IMPLEMENTED YET - PLEASE STOP/RESTART ALL CONTROLLERS MANUALLY FOR NOW** The controller manager detects that and stops all the controllers that are commanding that hardware and restarts broadcasters that are listening to its states.
