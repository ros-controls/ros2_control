Debugging
^^^^^^^^^

All controllers and hardware components are plugins loaded into the ``controller_manager``. Therefore, the debugger must be attached to the ``controller_manager``. If multiple ``controller_manager`` instances are running on your robot or machine, you need to attach the debugger to the ``controller_manager`` associated with the hardware component or controller you want to debug.

How-To
******************

* Install ``xterm``, ``gdb`` and ``gdbserver`` on your system

  .. code-block:: bash

    sudo apt install xterm gdb gdbserver

* Make sure you run a "debug" or "release with debug information" build:
  This is done by passing ``--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`` to ``colcon build``.
  Remember that in release builds some breakpoints might not behave as you expect as the the corresponding line might have been optimized by the compiler. For such cases, a full Debug build (``--cmake-args -DCMAKE_BUILD_TYPE=Debug``) is recommended.

* Adapt the launch file to run the controller manager with the debugger attached:

  * Version A: Run it directly with the gdb CLI:

    Add ``prefix=['xterm -e gdb -ex run --args']`` to the ``controller_manager`` node entry in your launch file.
    Due to how ``ros2launch`` works we need to run the specific node in a separate terminal instance.

  * Version B: Run it with gdbserver:

    Add ``prefix=['gdbserver localhost:3000']`` to the ``controller_manager`` node entry in your launch file.
    Afterwards, you can either attach a gdb CLI instance or any IDE of your choice to that ``gdbserver`` instance.
    Ensure you start your debugger from a terminal where you have sourced your workspace to properly resolve all paths.

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
        prefix=['xterm -e gdb -ex run --args']  # or prefix=['gdbserver localhost:3000']
    )

    ld.add_action(controller_manager)


Additional notes
*****************

* Debugging plugins

  You can only set breakpoints in plugins after the plugin has been loaded. In the ros2_control context this means after the controller / hardware component has been loaded:

* Debug builds

  It's often practical to include debug information only for the specific package you want to debug.
  ``colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`` or ``colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=Debug``

* Realtime

  .. warning::

    The ``update/on_activate/on_deactivate`` method of a controller and the ``read/write/on_activate/perform_command_mode_switch`` methods of a hardware component all run in the context of the realtime update loop. Setting breakpoints there can and will cause issues that might even break your hardware in the worst case.

  From experience, it might be better to use meaningful logs for the real-time context (with caution) or to add additional debug state interfaces (or publishers in the case of a controller).

However, running the controller_manager and your plugin with gdb can still be very useful for debugging errors such as segfaults, as you can gather a full backtrace.

References
***********

* `ROS 2 and GDB <https://juraph.com/miscellaneous/ros2_and_gdb/>`_
* `Using GDB to debug a plugin <https://stackoverflow.com/questions/10919832/how-to-use-gdb-to-debug-a-plugin>`_
* `GDB CLI Tutorial <https://www.cs.umd.edu/~srhuang/teaching/cmsc212/gdb-tutorial-handout.pdf>`_

Introspection and Debugging of the ros2_control setup
******************************************************

With the integration of the ``pal_statistics`` package, the ``controller_manager`` node publishes the registered variables within the same process to the ``~/introspection_data`` topics.
By default, all ``State`` and ``Command`` interfaces in the ``controller_manager`` are registered when they are added, and are unregistered when they are removed from the ``ResourceManager``.
The state of the all the registered entities are published at the end of every ``update`` cycle of the ``controller_manager``. For instance, In a complete synchronous ros2_control setup (with synchronous controllers and hardware components), this data in the ``Command`` interface is the command used by the hardware components to command the hardware.

All the registered variables are published over 3 topics: ``~/introspection_data/full``, ``~/introspection_data/names``, and ``~/introspection_data/values``.
- The ``~/introspection_data/full`` topic publishes the full introspection data along with names and values in a single message. This can be useful to track or view variables and information from command line.
- The ``~/introspection_data/names`` topic publishes the names of the registered variables. This topic contains the names of the variables registered. This is only published every time a a variables is registered and unregistered.
- The ``~/introspection_data/values`` topic publishes the values of the registered variables. This topic contains the values of the variables registered.

The topics ``~/introspection_data/full`` and ``~/introspection_data/values`` are always published on every update cycle asynchronously, provided that there is at least one subscriber to these topics.

The topic ``~/introspection_data/full`` can be used to integrate with your custom visualization tools or to track the variables from the command line. The topic ``~/introspection_data/names`` and ``~/introspection_data/values`` are to be used for visualization tools like `PlotJuggler <https://plotjuggler.io/>`_ to visualize the data.

.. note::
  If you have a high frequency of data, it is recommended to use the ``~/introspection_data/names`` and ``~/introspection_data/values`` topic. So, that the data transferred and stored is minimized.

How to introspect internal variables of controllers and hardware components
============================================================================

Any member variable of a controller or hardware component can be registered for the introspection. It is very important that the lifetime of this variable exists as long as the controller or hardware component is available.

.. note::
  If a variable's lifetime is not properly managed, it may be attempted to read, which in the worst case scenario will cause a segmentation fault.

How to register a variable for introspection
---------------------------------------------

1. Include the necessary headers in the controller or hardware component header file.

   .. code-block:: cpp

     #include <hardware_interface/introspection.hpp>

2. Register the variable in the configure method of the controller or hardware component.

   .. code-block:: cpp

     void MyController::on_configure()
     {
       ...
       // Register the variable for introspection
       REGISTER_ROS2_CONTROL_INTROSPECTION("my_variable_name", &my_variable_);
       ...
     }

3. By default, The introspection of all the registered variables of the controllers and the hardware components is only activated, when they are active and it is deactivated when the controller or hardware component is deactivated.

   .. code-block:: cpp

     void MyController::on_configure()
     {
       ...
       // Register the variable for introspection
       REGISTER_ROS2_CONTROL_INTROSPECTION("my_variable_name", &my_variable_, true);
       ...
     }

   .. note::
      If you want to keep the introspection active even when the controller or hardware component is not active, you can do that by calling ``this->enable_introspection(true)`` in the ``on_configure`` and ``on_deactivate`` method of the controller or hardware component after registering the variables.

Types of entities that can be introspected
-------------------------------------------

- Any variable that can be cast to a double is suitable for registration.
- A function that returns a value that can be cast to a double is also suitable for registration.
- Variables of complex structures can be registered by having defined introspection for its every internal variable.
- Introspection of custom types can be done by defining a `custom introspection function <https://github.com/pal-robotics/pal_statistics/blob/humble-devel/pal_statistics/include/pal_statistics/registration_utils.hpp>`_.

  .. note::
    Registering the variables for introspection is not real-time safe. It is recommended to register the variables in the ``on_configure`` method only.

Data Visualization
*******************

Data can be visualized with any tools that display ROS topics, but we recommend `PlotJuggler <https://plotjuggler.io/>`_ for viewing high resolution live data, or data in bags.

1. Open ``PlotJuggler`` running ``ros2 run plotjuggler plotjuggler``.
   .. image:: images/plotjuggler.png
2. Visualize the data:
   - Importing from the ros2bag
   - Subscribing to the ROS2 topics live with the ``ROS2 Topic Subscriber`` option under ``Streaming`` header.
3. Choose the topics ``~/introspection_data/names`` and ``~/introspection_data/values`` from the popup window.
   .. image:: images/plotjuggler_select_topics.png
4. Now, select the variables that are of your interest and drag them to the plot.
   .. image:: images/plotjuggler_visualizing_data.png
