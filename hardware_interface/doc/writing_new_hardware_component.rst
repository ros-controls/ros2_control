:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/writing_new_hardware_component.rst

.. _writing_new_hardware_component:

Writing a Hardware Component
============================

In ros2_control hardware system components are libraries, dynamically loaded by the controller manager using the `pluginlib <https://ros.org/wiki/pluginlib>`_ interface.
The following is a step-by-step guide to create source files, basic tests, and compile rules for a new hardware interface.

#. **Preparing package**

   If the package for the hardware interface does not exist, then create it first.
   The package should have ``ament_cmake`` as a build type.
   The easiest way is to search online for the most recent manual.
   A helpful command to support this process is ``ros2 pkg create``.
   Use the ``--help`` flag for more information on how to use it.
   There is also an option to create library source files and compile rules to help you in the following steps.

#. **Preparing source files**

   After creating the package, you should have at least ``CMakeLists.txt`` and ``package.xml`` files in it.
   Create also ``include/<PACKAGE_NAME>/`` and ``src`` folders if they do not already exist.
   In ``include/<PACKAGE_NAME>/`` folder add ``<robot_hardware_interface_name>.hpp`` and ``<robot_hardware_interface_name>.cpp`` in the ``src`` folder.
   Optionally add ``visibility_control.h`` with the definition of export rules for Windows.
   You can copy this file from an existing controller package and change the name prefix to the ``<PACKAGE_NAME>``.

#. **Adding declarations into header file (.hpp)**

   1. Take care that you use header guards. ROS2-style is using ``#ifndef`` and ``#define`` preprocessor directives. (For more information on this, a search engine is your friend :) ).

   2. Include ``"hardware_interface/$interface_type$_interface.hpp"`` and ``visibility_control.h`` if you are using one.
      ``$interface_type$`` can be ``Actuator``, ``Sensor`` or ``System`` depending on the type of hardware you are using. for more details about each type check :ref:`Hardware Components description <overview_hardware_components>`.

   3. Define a unique namespace for your hardware_interface. This is usually the package name written in ``snake_case``.

   4. Define the class of the hardware_interface, extending ``$InterfaceType$Interface``, e.g.,
      .. code:: c++
      class HardwareInterfaceName : public hardware_interface::$InterfaceType$Interface

   5. Add a constructor without parameters and the following public methods implementing ``LifecycleNodeInterface``: ``on_configure``, ``on_cleanup``, ``on_shutdown``, ``on_activate``, ``on_deactivate``, ``on_error``; and overriding ``$InterfaceType$Interface`` definition: ``on_init``, ``export_state_interfaces``, ``export_command_interfaces``, ``prepare_command_mode_switch`` (optional), ``perform_command_mode_switch`` (optional), ``read``, ``write``.

     For further explanation of hardware-lifecycle check the `pull request <https://github.com/ros-controls/ros2_control/pull/559/files#diff-2bd171d85b028c1b15b03b27d4e6dcbb87e52f705042bf111840e7a28ab268fc>`_ and for exact definitions of methods check the ``"hardware_interface/$interface_type$_interface.hpp"`` header or `doxygen documentation <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/api/namespacehardware__interface.html>`_ for *Actuator*, *Sensor* or *System*.

#. **Adding definitions into source file (.cpp)**

   #. Include the header file of your hardware interface and add a namespace definition to simplify further development.

   #. Implement ``on_init`` method. Here, you should initialize all member variables and process the parameters from the ``info`` argument.
      In the first line usually the parents ``on_init`` is called to process standard values, like name. This is done using: ``hardware_interface::(Actuator|Sensor|System)Interface::on_init(info)``.
      If all required parameters are set and valid and everything works fine return ``CallbackReturn::SUCCESS`` or ``return CallbackReturn::ERROR`` otherwise.

   #. Write the ``on_configure`` method where you usually setup the communication to the hardware and set everything up so that the hardware can be activated.

   #. Implement ``on_cleanup`` method, which does the opposite of ``on_configure``.
   #. ``Command-/StateInterfaces`` are now created and exported automatically by the framework via the ``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` methods based on the interfaces defined in the ``ros2_control`` XML-tag, which gets parsed and the ``InterfaceDescription`` is created accordingly (check the `hardware_info.hpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/hardware_info.hpp>`__).

      * To access the automatically created ``Command-/StateInterfaces`` we provide the ``std::unordered_map<std::string, InterfaceDescription>``, where the string is the fully qualified name of the interface and the ``InterfaceDescription`` is the configuration of the interface. The ``std::unordered_map<>`` are divided into ``type_state_interfaces_`` and ``type_command_interfaces_`` where type can be: ``joint``, ``sensor``, ``gpio`` and ``unlisted``. E.g. the ``CommandInterfaces`` for all joints can be found in the  ``joint_command_interfaces_`` map. The ``unlisted`` includes all interfaces not listed in the ``ros2_control`` XML-tag but were created by overriding the ``export_unlisted_command_interface_descriptions()`` or ``export_unlisted_state_interface_descriptions()`` function by creating some custom ``Command-/StateInterfaces``.
      * For the ``Sensor``-type hardware interface there is no ``export_command_interfaces`` method.
      * As a reminder, the full interface names have structure ``<joint_name>/<interface_type>``.

   #. (optional) If you want some unlisted ``Command-/StateInterfaces`` not included in the ``ros2_control`` XML-tag you can follow those steps:

      #. Override the ``virtual std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions()`` or ``virtual std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions()``
      #. Create the InterfaceDescription for each of the interfaces you want to create in the override ``export_unlisted_command_interface_descriptions()`` or ``export_unlisted_state_interface_descriptions()`` function, add it to a vector and return the vector:

         .. code-block:: c++

            std::vector<hardware_interface::InterfaceDescription> my_unlisted_interfaces;

            InterfaceInfo unlisted_interface;
            unlisted_interface.name = "some_unlisted_interface";
            unlisted_interface.min = "-5.0";
            unlisted_interface.data_type = "double";
            my_unlisted_interfaces.push_back(InterfaceDescription(info_.name, unlisted_interface));

            return my_unlisted_interfaces;

      #. The unlisted interface will then be stored in either the ``unlisted_command_interfaces_`` or ``unlisted_state_interfaces_`` map depending in which function they are created.
      #. You can access it like any other interface with the ``get_state(name)``, ``set_state(name, value)``, ``get_command(name)`` or ``set_command(name, value)``. E.g. ``get_state("some_unlisted_interface")``.

   #. (optional) In case the default implementation (``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` ) for exporting the ``Command-/StateInterfaces`` is not enough you can override them. You should however consider the following things:

      * If you want to have unlisted interfaces available you need to call the ``export_unlisted_command_interface_descriptions()`` or ``export_unlisted_state_interface_descriptions()`` and add them to the ``unlisted_command_interfaces_`` or ``unlisted_state_interfaces_``.
      * Don't forget to store the created ``Command-/StateInterfaces`` internally as you only return shared_ptrs and the resource manager will not provide access to the created ``Command-/StateInterfaces`` for the hardware. So you must take care of storing them yourself.
      * Names must be unique!

   #.  (optional) For *Actuator* and *System* types of hardware interface implement ``prepare_command_mode_switch`` and ``perform_command_mode_switch`` if your hardware accepts multiple control modes.

   #.  Implement the ``on_activate`` method where hardware "power" is enabled.

   #.  Implement the ``on_deactivate`` method, which does the opposite of ``on_activate``.

   #.  Implement ``on_shutdown`` method where hardware is shutdown gracefully.

   #.  Implement ``on_error`` method where different errors from all states are handled.

   #.  Implement the ``read`` method getting the states from the hardware and storing them to internal variables defined in ``export_state_interfaces``.

   #.  Implement ``write`` method that commands the hardware based on the values stored in internal variables defined in ``export_command_interfaces``.

   #.  IMPORTANT: At the end of your file after the namespace is closed, add the ``PLUGINLIB_EXPORT_CLASS`` macro.

      For this you will need to include the ``"pluginlib/class_list_macros.hpp"`` header.
      As first parameters you should provide exact hardware interface class, e.g., ``<my_hardware_interface_package>::<RobotHardwareInterfaceName>``, and as second the base class, i.e., ``hardware_interface::(Actuator|Sensor|System)Interface``.

#. **Writing export definition for pluginlib**

   #. Create the ``<my_hardware_interface_package>.xml`` file in the package and add a definition of the library and hardware interface's class which has to be visible for the pluginlib.
      The easiest way to do that is to check definition for mock components in the :ref:`hardware_interface mock_components <mock_components_userdoc>` section.

   #. Usually, the plugin name is defined by the package (namespace) and the class name, e.g.,
      ``<my_hardware_interface_package>/<RobotHardwareInterfaceName>``.
      This name defines the hardware interface's type when the resource manager searches for it.
      The other two parameters have to correspond to the definition done in the macro at the bottom of the ``<robot_hardware_interface_name>.cpp`` file.

#. **Writing a simple test to check if the controller can be found and loaded**

   #. Create the folder ``test`` in your package, if it does not exist already, and add a file named ``test_load_<robot_hardware_interface_name>.cpp``.

   #. You can copy the ``load_generic_system_2dof`` content defined in the `test_generic_system.cpp <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/test/mock_components/test_generic_system.cpp#L402-L407>`_ package.

   #. Change the name of the copied test and in the last line, where hardware interface type is specified put the name defined in ``<my_hardware_interface_package>.xml`` file, e.g., ``<my_hardware_interface_package>/<RobotHardwareInterfaceName>``.

#. **Add compile directives into ``CMakeLists.txt`` file**

   #. Under the line ``find_package(ament_cmake REQUIRED)`` add further dependencies.
      Those are at least: ``hardware_interface``, ``pluginlib``, ``rclcpp`` and ``rclcpp_lifecycle``.

   #. Add a compile directive for a shared library providing the ``<robot_hardware_interface_name>.cpp`` file as the source.

   #. Add targeted include directories for the library. This is usually only ``include``.

   #. Add ament dependencies needed by the library. You should add at least those listed under 1.

   #. Export for pluginlib description file using the following command:
      .. code:: cmake

         pluginlib_export_plugin_description_file(hardware_interface <my_hardware_interface_package>.xml)

   #. Add install directives for targets and include directory.

   #. In the test section add the following dependencies: ``ament_cmake_gmock``, ``hardware_interface``.

   #. Add compile definitions for the tests using the ``ament_add_gmock`` directive.
      For details, see how it is done for mock hardware in the `ros2_control <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/CMakeLists.txt>`_ package.

   #. (optional) Add your hardware interface`s library into ``ament_export_libraries`` before ``ament_package()``.

#. **Add dependencies into ``package.xml`` file**

   #. Add at least the following packages into ``<depend>`` tag: ``hardware_interface``, ``pluginlib``, ``rclcpp``, and ``rclcpp_lifecycle``.

   #. Add at least the following package into ``<test_depend>`` tag: ``ament_add_gmock`` and ``hardware_interface``.

#.  **Compiling and testing the hardware component**

   #. Now everything is ready to compile the hardware component using the ``colcon build <my_hardware_interface_package>`` command.
      Remember to go into the root of your workspace before executing this command.

   #. If compilation was successful, source the ``setup.bash`` file from the install folder and execute ``colcon test <my_hardware_interface_package>`` to check if the new controller can be found through ``pluginlib`` library and be loaded by the controller manager.


That's it! Enjoy writing great controllers!


Useful External References
---------------------------

- `Templates and scripts for generating controllers shell <https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros2_control/setup_robot_hardware_interface.html>`_

  .. NOTE:: The script is currently only recommended to use for Foxy, not compatible with the API from Galactic and onwards.
