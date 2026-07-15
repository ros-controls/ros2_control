:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/monitoring_and_tuning.rst

.. _monitoring_and_tuning:

Monitoring and Tuning
======================

ros2_control ``controller_interface`` has a ``ControllerUpdateStats`` structure which can be used to monitor the controller update rate and the missed update cycles. The data is published to the ``/diagnostics`` and also ``/controller_manager/introspection_data/*`` topics. This can be used to fine tune the controller update rate.
