:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/error_and_warning_interfaces_userdoc.rst

.. _error_and_warning_interfaces_userdoc:

Error and Warning Interfaces
============================

By default we now create the following error and warning interfaces:

+-----------------+--------------+----------------------------------------------------------------------------------------------------------------------+
| Type            | Datatype     | Description                                                                                                          |
+=================+====================+================================================================================================================+
| Emergency Stop  | Bool               | Used for signaling that hardwares emergency stop is active. Only for Actuator and System.                      |
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
| Error Code      | array<uint8_t, 32> | Used for sending 32 error codes (uint8_t) at the same time.                                                    |
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
| Error Message   | array<string, 32>  | Used for sending 32 error messages where the message at position x corresponds to error code at position x.    |
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
| Warning Code    | array<int8_t, 32>  | Used for sending 32 Warning codes (int8_t) at the same time.                                                   |
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+
| Warning Message | array<string, 32>  | Used for sending 32 warning messages where the message at position x corresponds to warning code at position x.|
+-----------------+--------------------+----------------------------------------------------------------------------------------------------------------+

The error and warning interfaces are created as ``StateInterfaces`` and are stored inside the Actuator-, Sensor- or SystemInterface. They can be accessed via getter and setter methods. E.g. if you want to get/set the emergency stop signal you can do so with the ``get_emergency_stop()`` or ``set_emergency_stop(const bool & emergency_stop)`` methods. For the error and warning signals similar getters and setters exist.

Note: The SensorInterface does not have a Emergency Stop interface.
