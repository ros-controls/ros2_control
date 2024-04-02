:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/ros2controlcli/doc/userdoc.rst

.. _ros2controlcli_userdoc:

Command Line Interface
======================

The following commands support interacting with the ``controller_manager`` from the command line. They are available through the standard ROS2 CLI framework.

Currently supported commands are

    - ros2 control list_controllers
    - ros2 control list_controller_types
    - ros2 control list_hardware_components
    - ros2 control list_hardware_interfaces
    - ros2 control load_controller
    - ros2 control reload_controller_libraries
    - ros2 control set_controller_state
    - ros2 control set_hardware_component_state
    - ros2 control switch_controllers
    - ros2 control unload_controller
    - ros2 control view_controller_chains


list_controllers
----------------

.. code-block:: console

    $ ros2 control list_controllers -h
    usage: ros2 control list_controllers [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

    Output the list of loaded controllers, their type and status

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

Example output:

.. code-block:: console

    $ ros2 control list_controllers
    test_controller_name[test_controller]    active


list_controller_types
---------------------

.. code-block:: console

    $ ros2 control list_controller_types -h
    usage: ros2 control list_controller_types [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

    Output the available controller types and their base classes

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

Example output:

.. code-block:: console

    $ ros2 control list_controller_types
    diff_drive_controller/DiffDriveController                              controller_interface::ControllerInterface
    joint_state_broadcaster/JointStateBroadcaster                          controller_interface::ControllerInterface
    joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface


list_hardware_components
------------------------

.. code-block:: console

    $ ros2 control list_hardware_components -h
    usage: ros2 control list_hardware_components [-h] [--spin-time SPIN_TIME] [-s] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

    Output the list of available hardware components

    options:
    -h, --help            show this help message and exit
    --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
    -s, --use-sim-time    Enable ROS simulation time
    --verbose, -v         List hardware components with command and state interfaces
    -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
    --include-hidden-nodes
                            Consider hidden nodes as well


Example output:

.. code-block:: console

    $ ros2 control list_hardware_components
    Hardware Component 0
        name: RRBot
        type: system
        plugin name: ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware
        state: id=3 label=active


list_hardware_interfaces
------------------------

.. code-block:: console

    $ ros2 control list_hardware_interfaces -h
    usage: ros2 control list_hardware_interfaces [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

    Output the list of available command and state interfaces

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well


.. code-block:: console

    $ ros2 control list_hardware_interfaces
    command interfaces
      joint1/position [unclaimed]
      joint2/position [unclaimed]
    state interfaces
      joint1/position
      joint2/position


load_controller
---------------

.. code-block:: console

    $ ros2 control load_controller -h
    usage: ros2 control load_controller [-h] [--spin-time SPIN_TIME] [-s] [--set-state {configured,inactive,active}] [-c CONTROLLER_MANAGER]
                                        [--include-hidden-nodes]
                                        controller_name

    Load a controller in a controller manager

    positional arguments:
      controller_name       Name of the controller

    options:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -s, --use-sim-time    Enable ROS simulation time
      --set-state {configured,inactive,active}
                            Set the state of the loaded controller
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

reload_controller_libraries
---------------------------

.. code-block:: console

    $ ros2 control reload_controller_libraries -h
    usage: ros2 control reload_controller_libraries [-h] [--spin-time SPIN_TIME] [--force-kill] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

    Reload controller libraries

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      --force-kill          Force stop of loaded controllers
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

set_controller_state
--------------------

.. code-block:: console

    $ ros2 control set_controller_state -h
    usage: ros2 control set_controller_state [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name {inactive,active}

    Adjust the state of the controller

    positional arguments:
      controller_name       Name of the controller to be changed
      {inactive,active}
                            State in which the controller should be changed to

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

set_hardware_component_state
----------------------------

.. code-block:: console

    $ ros2 control set_hardware_component_state -h
    usage: ros2 control set_hardware_component_state [-h] [--spin-time SPIN_TIME] [-s] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]
                                                    hardware_component_name {unconfigured,inactive,active}

    Adjust the state of the hardware component

    positional arguments:
      hardware_component_name
                            Name of the hardware_component to be changed
      {unconfigured,inactive,active}
                            State in which the hardware component should be changed to

    options:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -s, --use-sim-time    Enable ROS simulation time
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

switch_controllers
------------------

.. code-block:: console

    $ ros2 control switch_controllers -h
    usage: ros2 control switch_controllers [-h] [--spin-time SPIN_TIME] [--deactivate [CTRL1 [CTRL2 ...]]] [--activate [CTRL1 [CTRL2 ...]]] [--strict] [--activate-asap] [--switch-timeout SWITCH_TIMEOUT] [-c CONTROLLER_MANAGER]
                                          [--include-hidden-nodes]

    Switch controllers in a controller manager

    optional arguments:
    -h, --help            show this help message and exit
    --spin-time SPIN_TIME
    Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
    --deactivate [CTRL1 [CTRL2 ...]]
    Name of the controllers to be deactivate
    --activate [CTRL1 [CTRL2 ...]]
    Name of the controllers to be activated
    --strict              Strict switch
    --activate-asap       Activate asap controllers
    --switch-timeout SWITCH_TIMEOUT
    Timeout for switching controllers
    -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
    Name of the controller manager ROS node
    --include-hidden-nodes
                          Consider hidden nodes as well

unload_controller
-----------------

.. code-block:: console

    $ ros2 control unload_controller -h
    usage: ros2 control unload_controller [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name

    Unload a controller in a controller manager

    positional arguments:
      controller_name       Name of the controller

    optional arguments:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well

view_controller_chains
----------------------

.. code-block:: console

    $ ros2 control view_controller_chains -h
    usage: ros2 view_controller_chains

    Generates a diagram of the loaded chained controllers into /tmp/controller_diagram.gv.pdf

    options:
      -h, --help            show this help message and exit
      --spin-time SPIN_TIME
                            Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
      -s, --use-sim-time    Enable ROS simulation time
      -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                            Name of the controller manager ROS node
      --include-hidden-nodes
                            Consider hidden nodes as well
