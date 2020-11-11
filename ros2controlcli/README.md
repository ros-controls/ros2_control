# ros2controlcli
Command line interface for controller manager

## Verbs 


### list

```bash
$ ros2 control list -h
usage: ros2 control list [-h] [--spin-time SPIN_TIME] [--no-daemon] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

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
$ ros2 control list 
test_controller_name[test_controller]    active 
```

### list_types

```bash
$ ros2 control list_types -h
usage: ros2 control list_types [-h] [--spin-time SPIN_TIME] [--no-daemon] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

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
$ ros2 control list_types 
diff_drive_controller/DiffDriveController                              controller_interface::ControllerInterface
joint_state_controller/JointStateController                            controller_interface::ControllerInterface
joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface
test_controller      
```

### load

```bash
$ ros2 control load -h
usage: ros2 control load [-h] [--spin-time SPIN_TIME] [--no-daemon] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name

Load a controller in a controller manager

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

### reload_libraries

```bash
$ ros2 control reload_libraries -h
usage: ros2 control reload_libraries [-h] [--spin-time SPIN_TIME] [--force-kill] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

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

### switch

```bash
$ ros2 control switch -h
usage: ros2 control switch [-h] [--spin-time SPIN_TIME] [--no-daemon] [--stop-controllers [STOP_CONTROLLERS [STOP_CONTROLLERS ...]]]
                           [--start-controllers [START_CONTROLLERS [START_CONTROLLERS ...]]] [--strict] [--start-asap]
                           [--switch-timeout SWITCH_TIMEOUT] [-c CONTROLLER_MANAGER] [--include-hidden-nodes]

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





### unload

```bash
$ ros2 control unload -h
usage: ros2 control unload [-h] [--spin-time SPIN_TIME] [--no-daemon] [-c CONTROLLER_MANAGER] [--include-hidden-nodes] controller_name

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