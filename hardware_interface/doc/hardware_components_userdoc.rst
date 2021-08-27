.. _hardware_components_userdoc:

Hardware Components
-------------------
Hardware components represent abstraction of physical hardware in ros2_control framework.
There are three types of hardware Actuator, Sensor and System.
For details on each type check `Hardware Components description <https://ros-controls.github.io/control.ros.org/getting_started.html#hardware-components>`_.


Migration from Foxy to Galactic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Between Foxy and Galactic we did substantial changes to interface of hardware components to enable management of their lifecycle.
The following list shows mandatory changes when porting existing hardware components to Galactic:

1. Rename ``configure`` to ``on_init`` and change return type to ``CallbackReturn``
1. If using BaseInterface then replace first thee lines in ``on_init`` to:

.. code-block:: c++

   if (on_init_default(info) != CallbackReturn::SUCCESS)
   {
     return CallbackReturn::ERROR;
   }

1. Change last return of ``on_init`` to ``return CallbackReturn::SUCCESS;``;
1. Remove all lines with ``status_ = ...`` or ``status::...``
1. Rename ``start()`` to ``on_activate()`` and ``stop()`` to ``on_deactivate()``
1. Change return type of ``on_activate`` and ``on_deactivate`` to ``CallbackReturn``
1. Change last return of ``on_activate`` and ``on_deactivate`` to ``return CallbackReturn::SUCCESS;``
1. If you have any ``return_type::ERROR`` in ``on_init``, ``on_activate``, or ``in_deactivate`` change to ``CallbackReturn::ERROR``
