:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _ros2_control_framework:

#################
ros2_control
#################

This is the documentation of the ros2_control framework core.

`GitHub Repository <https://github.com/ros-controls/ros2_control>`_

=================
API Documentation
=================

API documentation is parsed by doxygen and can be found `here <../../api/index.html>`_

==================
Controller Manager
==================

.. toctree::
   :titlesonly:

   Controller Manager <../controller_manager/doc/userdoc>

========
Concepts
========

.. toctree::
   :titlesonly:

   Controller Chaining / Cascade Control <../controller_manager/doc/controller_chaining>
   Joint Kinematics <../hardware_interface/doc/joints_userdoc>
   Joint Limiting <../hardware_interface/doc/joint_limiting>
   Hardware Components <../hardware_interface/doc/hardware_components_userdoc>
   Mock Components <../hardware_interface/doc/mock_components_userdoc>
   Support for Asynchronous Updates <../controller_manager/doc/async_updates>
   Different Clocks used by Controller Manager <../controller_manager/doc/clocks>

=============================
Guidelines and Best Practices
=============================

.. toctree::
   :titlesonly:

   Debugging the Controller Manager and Plugins <debugging>
   Introspecting Controllers and Hardware Components <introspection>
   Monitoring and Tuning <monitoring_and_tuning>
