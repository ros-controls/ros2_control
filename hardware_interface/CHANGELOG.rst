^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2022-08-03)
-------------------
* Error if a hardware name is duplicated (backport `#672 <https://github.com/ros-controls/ros2_control/issues/672>`_) (`#707 <https://github.com/ros-controls/ros2_control/issues/707>`_)
* Contributors: Andy Zelenak, Bence Magyar

0.10.1 (2022-05-31)
-------------------

0.10.0 (2022-02-23)
-------------------

0.9.0 (2021-12-20)
------------------

0.8.1 (2021-10-25)
------------------
* add M_PI macro for windows foxy in test_component_parser.cpp (`#545 <https://github.com/ros-controls/ros2_control/issues/545>`_)
* Extend GenericSystem by adding mapping of position with offset to custom interface. (`#469 <https://github.com/ros-controls/ros2_control/issues/469>`_) (`#523 <https://github.com/ros-controls/ros2_control/issues/523>`_)
* Contributors: BenjaminHug8, Denis Štogl

0.8.0 (2021-08-28)
------------------
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Fixup doc typo (`#492 <https://github.com/ros-controls/ros2_control/issues/492>`_)
* Add docs for fake components (`#466 <https://github.com/ros-controls/ros2_control/issues/466>`_)
* sort interfaces in resource manager (`#483 <https://github.com/ros-controls/ros2_control/issues/483>`_)
* fix format (`#484 <https://github.com/ros-controls/ros2_control/issues/484>`_)
* Transmission parsing v2 (`#471 <https://github.com/ros-controls/ros2_control/issues/471>`_)
  * move parsing responsibility to hardware_interface
  * parse transmission type
  * Cleanup unused parser
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Extended GenericSystem with state offset options for testing some special control cases. (`#350 <https://github.com/ros-controls/ros2_control/issues/350>`_)
  * Extended GenericSystem with state offset options for testing some special control cases.
  * Better parameter name
  * Apply offset only to position interfaces.
* Added GPIO parsing and test (`#436 <https://github.com/ros-controls/ros2_control/issues/436>`_)
* Fixes for windows (`#443 <https://github.com/ros-controls/ros2_control/issues/443>`_)
  * Fix building on windows
  * Fix MSVC linker error when building tests
  * Fix hang when loading controller on windows
  * Use better log for configuring controller
  * Be consistent with visibility control
  * Use try_lock throw exception on failure
* Contributors: Akash, Bence Magyar, Denis Štogl, Karsten Knese, Mathias Arbo, Jafar Abdi

0.7.1 (2021-06-15)
------------------
* [FakeSystem] Set default command interface to NaN (`#424 <https://github.com/ros-controls/ros2_control/issues/424>`_)
* Contributors: Denis Štogl, Bence Magyar

0.7.0 (2021-06-06)
------------------
* Add FTS as first semantic components to simplify controllers. (`#370 <https://github.com/ros-controls/ros2_control/issues/370>`_)
* Contributors: bailaC, Denis Štogl, Jordan Palacios

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------
* Remove the with_value_ptr and class templatization for ReadOnlyHandle (`#379 <https://github.com/ros-controls/ros2_control/issues/379>`_)
* fake_components: Add mimic joint to generic system (`#409 <https://github.com/ros-controls/ros2_control/issues/409>`_)
* List controller claimed interfaces (`#407 <https://github.com/ros-controls/ros2_control/issues/407>`_)
* Contributors: El Jawad Alaa, Jafar Abdi, Jordan Palacios, Bence Magyar

0.5.0 (2021-05-03)
------------------
* Make hardware interface types as const char array rather than const char pointer (`#408 <https://github.com/ros-controls/ros2_control/issues/408>`_)
* use auto instead of uint (`#398 <https://github.com/ros-controls/ros2_control/issues/398>`_)
* hardware_interface mode switching using prepareSwitch doSwitch approach (`#348 <https://github.com/ros-controls/ros2_control/issues/348>`_)
* avoid deprecations (`#393 <https://github.com/ros-controls/ros2_control/issues/393>`_)
* move deprecation note before function definition instead of inside (`#381 <https://github.com/ros-controls/ros2_control/issues/381>`_)
* Replace standard interfaces' hard-coded strings by constants (`#376 <https://github.com/ros-controls/ros2_control/issues/376>`_)
* add deprecation note for with_value_ptr (`#378 <https://github.com/ros-controls/ros2_control/issues/378>`_)
* Contributors: El Jawad Alaa, Jafar Abdi, Karsten Knese, Mateus Amarante, Mathias Hauan Arbo, Bence Magyar

0.4.0 (2021-04-07)
------------------
* [ros2_control_test_assets] Fix typo (`#371 <https://github.com/ros-controls/ros2_control/issues/371>`_)
* uint -> size_t, 0u and auto (`#346 <https://github.com/ros-controls/ros2_control/issues/346>`_)
* Contributors: Karsten Knese, Yutaka Kondo

0.3.0 (2021-03-21)
------------------
* Capatalized error message and put the controllers name and resource name inside quote (`#338 <https://github.com/ros-controls/ros2_control/issues/338>`_)
* Parse True and true in fakesystem, touch up variable name
* Contributors: Denis Štogl, suab321321

0.2.1 (2021-03-02)
------------------
* Remove unused include (`#336 <https://github.com/ros-controls/ros2_control/issues/336>`_)
* Contributors: Bence Magyar

0.2.0 (2021-02-26)
------------------
* Add "Fake" components for simple integration of framework (`#323 <https://github.com/ros-controls/ros2_control/issues/323>`_)
* Contributors: Denis Štogl

0.1.6 (2021-02-05)
------------------
* correct hardware interface validation in resource manager. (`#317 <https://github.com/ros-controls/ros2_control/issues/317>`_)
* Contributors: Karsten Knese

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------
* Add test assets package (`#289 <https://github.com/ros-controls/ros2_control/issues/289>`_)
* update doxygen style according to ros2 core standard (`#300 <https://github.com/ros-controls/ros2_control/issues/300>`_)
* Move test_components from test_robot_hardware to hardware_interface package (`#288 <https://github.com/ros-controls/ros2_control/issues/288>`_)
* Contributors: Denis Štogl, João Victor Torres Borges

0.1.3 (2021-01-21)
------------------

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
* Added starting of resources into CM and RM (`#240 <https://github.com/ros-controls/ros2_control/issues/240>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* Use constants instead of strings in tests (`#241 <https://github.com/ros-controls/ros2_control/issues/241>`_)
* resource loaning (`#224 <https://github.com/ros-controls/ros2_control/issues/224>`_)
* Allocate memory for components and handles (`#207 <https://github.com/ros-controls/ros2_control/issues/207>`_)
* rename command/state handles to command/state interfaces (`#223 <https://github.com/ros-controls/ros2_control/issues/223>`_)
* Remodel component interfaces (`#203 <https://github.com/ros-controls/ros2_control/issues/203>`_)
* adapt component parser to new xml schema (`#209 <https://github.com/ros-controls/ros2_control/issues/209>`_)
* remove logical components, move hardware resources (`#201 <https://github.com/ros-controls/ros2_control/issues/201>`_)
* Replace rclcpp by rcutils logging tools in hardware_interface pkg (`#205 <https://github.com/ros-controls/ros2_control/issues/205>`_)
* Add a struct for Interface information, update the test URDF (`#167 <https://github.com/ros-controls/ros2_control/issues/167>`_)
* Add virtual modifier to the functions of Joint and Sensor component (`#178 <https://github.com/ros-controls/ros2_control/issues/178>`_)
* Hide component parser api (`#157 <https://github.com/ros-controls/ros2_control/issues/157>`_)
* Remove old joint state and joint command handles (`#134 <https://github.com/ros-controls/ros2_control/issues/134>`_)
* New version of component parser (`#127 <https://github.com/ros-controls/ros2_control/issues/127>`_)
* Dynamic joint handles (`#125 <https://github.com/ros-controls/ros2_control/issues/125>`_)
* Hardware component interfaces (`#121 <https://github.com/ros-controls/ros2_control/issues/121>`_)
* Add ActuatorHandle and Implement string-based interface handle-handling using DynamicJointState message (`#112 <https://github.com/ros-controls/ros2_control/issues/112>`_)
* Change Hardware return type to enum class (`#114 <https://github.com/ros-controls/ros2_control/issues/114>`_)
* Replace RCUTILS\_ with RCLCPP\_ for logging (`#62 <https://github.com/ros-controls/ros2_control/issues/62>`_)
* import hardware_interface
* Contributors: Andreas Klintberg, Andy Zelenak, Bence Magyar, Colin MacKenzie, Denis Štogl, Jafar Abdi, Jordan Palacios, Karsten Knese, Mateus Amarante, Matthew Reynolds, Victor Lopez, Yutaka Kondo
