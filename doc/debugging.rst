Debugging
^^^^^^^^^

All controllers and hardware interfaces are plugins that are loaded into the ``controller_manager``. Therefore the debugger
needs to be attached to the controller_manager

How-To
******************

* Install xterm, gdb and gdbserver on your system

  .. code-block:: bash

    sudo apt install xterm gdb gdbserver
* Make sure you run a debug/release with debug information build: 
  This is done by passing ``--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`` to colcon build.
  Remember that in Release builds some breakpoints might not behave as you think as the the corresponding line might have been removed by the optimizer. For certain cases a full Debug build (``--cmake-args -DCMAKE_BUILD_TYPE=Debug``)

* Adapt the launch file to run the controller manager with the debugger attached:

  * Version A: Run it directly with the gdb cli:

    Add ``prefix=['xterm -e gdb -ex run --args']`` to the ``controller_manager`` node entry in your launch file.
    Due to the way ros2launch works we need to run the specific node in a separate terminal instance. 
    See a gdb cli tutorial for further steps

  * Version B: Run it with gdbserver:

    Add ``prefix=['gdbserver localhost:3000']`` to the ``controller_manager`` node entry in your launch file.
    Afterwards you can either attach a gdb cli instance or any IDE of your choice to that gdbserver instance. 
    You probably want to make sure to start your debugger from a terminal where you sourced your workspace in order to have to find all paths

  Example launch file entry:

  .. code-block:: python

    # Obtain the controller config file for the ros2 control node
    controller_config_file = get_package_file("<package name>", "config/controllers.yaml")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        output="both",
        emulate_tty=True,
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
        prefix=['xterm -e gdb -ex run --args']`  # or prefix=['gdbserver localhost:3000']
    )

    ld.add_action(controller_manager)


Additional notes
*****************

* Debugging plugins

  You can only set breakpoints in plugins after the plugin has been loaded. In the ros2control context this means after the controller / hardware interface has been loaded:

* Debug builds

  It often makes sense to only build the package you want to debug with debug information. 
  ``colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`` or ``colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=Debug``

* Realtime

  The ``update/on_activate`` method of a controller and the ``read/write/on_activate/perform_command_mode_switch`` methods of a hardware interface all run in the context
  of the realtime update loop. Setting breakpoints there can and will cause issues which might even break your hardware in the worst case.
  
  From experience it might be better to either use (also just carefully) std::cout debugging the realtime context or add additional 
  debug state interfaces (or publishers in case of a controller) for debugging. 

  Running the controller_manager + your plugin with gdb can nevertheless we really useful for debugging errors such as segfaults as you can gather a full ``backtrace``

