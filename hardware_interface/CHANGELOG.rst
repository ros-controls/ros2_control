^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
