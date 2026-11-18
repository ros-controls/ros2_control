:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/hardware_components_userdoc.rst

.. _hardware_components_userdoc:

Hardware Components
-------------------
Hardware components represent abstraction of physical hardware in ros2_control framework.
There are three types of hardware Actuator, Sensor and System.
For details on each type check :ref:`overview_hardware_components` description.

.. toctree::
   :titlesonly:

   Hardware Interface Types <hardware_interface_types_userdoc>
   Writing a Hardware Component <writing_new_hardware_component>
   Different Update Rates <different_update_rates_userdoc>
   Asynchronous Execution <asynchronous_components>
   Semantic Components <semantic_components>
   Mock Components <mock_components_userdoc>
   Lifecycle of a Hardware Component <lifecycle_of_a_hardware_component>
   Handling of Errors During read() and write() Calls <handling_errors_during_read_write>
