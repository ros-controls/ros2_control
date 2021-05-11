# ros2controlcli
Command line interface for controller manager

## Verbs 

### configure_controller

```bash
$ ros2 control configure_controller -h
usage: ros2 control configure_controller [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name

Configure a controller in a controller manager

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
```

### list_controllers

```bash
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
```

```bash
$ ros2 control list_controllers
test_controller_name[test_controller]    active 
```

### list_controller_types

```bash
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
```

```bash
$ ros2 control list_controller_types 
diff_drive_controller/DiffDriveController                              controller_interface::ControllerInterface
joint_state_controller/JointStateController                            controller_interface::ControllerInterface
joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface
```

### list_hardware_interfaces

```bash
$ ros2 control list_hardware_interfaces -h
usage: ros2 control list_hardware_interfaces [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

Output the list of loaded controllers, their type and status

optional arguments:
  -h, --help            show this help message and exit
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --include-hidden-nodes
                        Consider hidden nodes as well
```

```bash
$ ros2 control list_hardware_interfaces
command interfaces
	joint1/position [unclaimed]
	joint2/position [unclaimed]
state interfaces
	 joint1/position
	 joint2/position

```

### load_controller

```bash
$ ros2 control load_controller -h
usage: ros2 control load_controller [-h] [--spin-time SPIN_TIME] [--set_state {configure,start}] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name

Load a controller in a controller manager

positional arguments:
  controller_name       Name of the controller

optional arguments:
  -h, --help            show this help message and exit
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
  --set_state {configure,start}
                        Set the state of the loaded controller
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --include-hidden-nodes
                        Consider hidden nodes as well
```

### reload_controller_libraries

```bash
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
```

### set_controller_state

```bash
$ ros2 control set_controller_state -h
usage: ros2 control set_controller_state [-h] [--spin-time SPIN_TIME] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name {configure,start,stop}

Adjust the state of the controller

positional arguments:
  controller_name       Name of the controller to be changed
  {configure,start,stop}
                        State in which the controller should be changed to

optional arguments:
  -h, --help            show this help message and exit
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --include-hidden-nodes
                        Consider hidden nodes as well
```

### switch_controllers

```bash
$ ros2 control switch_controllers -h
usage: ros2 control switch_controllers [-h] [--spin-time SPIN_TIME] [--stop-controllers [STOP_CONTROLLERS [STOP_CONTROLLERS ...]]] [--start-controllers [START_CONTROLLERS [START_CONTROLLERS ...]]] [--strict]
                                       [--start-asap] [--switch-timeout SWITCH_TIMEOUT] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

Switch controllers in a controller manager

optional arguments:
  -h, --help            show this help message and exit
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
  --stop-controllers [STOP_CONTROLLERS [STOP_CONTROLLERS ...]]
                        Name of the controllers to be stopped
  --start-controllers [START_CONTROLLERS [START_CONTROLLERS ...]]
                        Name of the controllers to be started
  --strict              Strict switch
  --start-asap          Start asap controllers
  --switch-timeout SWITCH_TIMEOUT
                        Timeout for switching controllers
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --include-hidden-nodes
                        Consider hidden nodes as well
```





### unload_controller

```bash
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
```