:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/handling_errors_during_read_write.rst

Handling of Errors During read() and write() Calls
--------------------------------------------------
If ``hardware_interface::return_type::ERROR`` is returned from ``read()`` or ``write()`` methods of a hardware_interface class, ``on_error(previous_state)`` method will be called to handle any error that happened.

Error handling follows the `node lifecycle <https://design.ros2.org/articles/node_lifecycle.html>`_.
If successful ``CallbackReturn::SUCCESS`` is returned and hardware is again in ``UNCONFIGURED`` state, if any ``ERROR`` or ``FAILURE`` happens the hardware ends in ``FINALIZED`` state and can not be recovered.
The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.
