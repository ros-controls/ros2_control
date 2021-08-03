.. _framework:

=====================
ROS2 Control API
=====================

`GitHub Repository <https://github.com/ros-controls/ros2_control>`_

Controller Interface
====================
.. doxygenfile:: controller_interface.hpp
.. doxygenfile:: controller_state_names.hpp
.. doxygenfile:: helpers.hpp
.. doxygenfile:: controller_interface/visibility_control.h

Semantic Components
*******************
.. doxygenfile:: force_torque_sensor.hpp
.. doxygenfile:: imu_sensor.hpp
.. doxygenfile:: semantic_component_interface.hpp

Controller Manager
==================
.. won't build with hpp
.. doxygenfile:: controller_manager.cpp
.. won't build with controller_spec.hpp
.. doxygenfile:: controller_manager/visibility_control.h

Hardware Interface
==================

.. doxygenfile:: actuator.hpp
.. doxygenfile:: controller_info.hpp
.. doxygenfile:: loaned_state_interface.hpp
.. doxygenfile:: sensor_interface.hpp
.. doxygenfile:: hardware_interface/visibility_control.h
.. doxygenfile:: actuator_interface.hpp
.. doxygenfile:: hardware_interface/handle.hpp
.. doxygenfile:: macros.hpp
.. doxygenfile:: system.hpp
.. doxygenfile:: base_interface.hpp
.. doxygenfile:: hardware_info.hpp
.. doxygenfile:: resource_manager.hpp
.. doxygenfile:: system_interface.hpp
.. doxygenfile:: component_parser.hpp
.. doxygenfile:: loaned_command_interface.hpp
.. doxygenfile:: sensor.hpp

Fake Components
***************
.. doxygenfile:: generic_system.hpp
.. doxygenfile:: fake_components/visibility_control.h

Joint Limits Interface
======================
.. doxygenfile:: joint_limits.hpp
.. doxygenfile:: joint_limits_interface.hpp
.. doxygenfile:: joint_limits_interface_exception.hpp
.. doxygenfile:: joint_limits_rosparam.hpp
.. doxygenfile:: joint_limits_urdf.hpp

ROS2 Control Test Assets
========================
.. doxygenfile:: components_urdfs.hpp
.. doxygenfile:: descriptions.hpp

Transmission Interface
======================
.. doxygenfile:: accessor.hpp
.. doxygenfile:: four_bar_linkage_transmission.hpp
.. doxygenfile:: transmission.hpp
.. doxygenfile:: transmission_interface/visibility_control.h
.. doxygenfile:: differential_transmission.hpp
.. doxygenfile:: transmission_interface/handle.hpp
.. doxygenfile:: transmission_info.hpp
.. doxygenfile:: exception.hpp
.. doxygenfile:: simple_transmission.hpp
.. doxygenfile:: transmission_parser.hpp
