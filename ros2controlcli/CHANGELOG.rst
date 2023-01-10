^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2controlcli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.5.1 (2023-01-06)
------------------

3.5.0 (2022-12-06)
------------------
* Fix hardware interface CLI description (`#864 <https://github.com/ros-controls/ros2_control/issues/864>`_)
* Contributors: Christoph Fröhlich

3.4.0 (2022-11-27)
------------------

3.3.0 (2022-11-15)
------------------

3.2.0 (2022-10-15)
------------------

3.1.0 (2022-10-05)
------------------

3.0.0 (2022-09-19)
------------------

2.15.0 (2022-09-19)
-------------------
* migrate from graphviz python to pygraphviz (`#812 <https://github.com/ros-controls/ros2_control/issues/812>`_)
* Contributors: Sachin Kumar

2.14.0 (2022-09-04)
-------------------
* Visualize chained controllers with graphviz (`#763 <https://github.com/ros-controls/ros2_control/issues/763>`_)
* Corrected the site link to a valid one. (`#801 <https://github.com/ros-controls/ros2_control/issues/801>`_)
* Contributors: Interactics, Paul Gesel

2.13.0 (2022-08-03)
-------------------
* Add chained controllers information in list controllers service #abi-braking (`#758 <https://github.com/ros-controls/ros2_control/issues/758>`_)
  * add chained controllers in ros2controlcli
  * remove controller_group from service
  * added comments to ControllerState message
  * added comments to ChainedConnection message
* Added spawner colors to command interfaces based on availablity and claimed status (`#754 <https://github.com/ros-controls/ros2_control/issues/754>`_)
* Contributors: Leander Stephen D'Souza, Paul Gesel

2.12.1 (2022-07-14)
-------------------

2.12.0 (2022-07-09)
-------------------
* Deprecate and rename `start` and `stop` nomenclature toward user to `activate` and `deactivate` #ABI-breaking (`#755 <https://github.com/ros-controls/ros2_control/issues/755>`_)
  * Rename fields and deprecate old nomenclature.
  * Add new defines to SwitchController.srv
  * Deprecated start/stop nomenclature in all CLI commands.
  * Deprecate 'start_asap' too as other fields.
* Contributors: Denis Štogl

2.11.0 (2022-07-03)
-------------------
* Remove hybrid services in controller manager. They are just overhead. (`#761 <https://github.com/ros-controls/ros2_control/issues/761>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Add available status and moved to fstrings when listing hardware interfaces (`#739 <https://github.com/ros-controls/ros2_control/issues/739>`_)
* Contributors: Bence Magyar, Denis Štogl, Leander Stephen D'Souza

2.10.0 (2022-06-18)
-------------------

2.9.0 (2022-05-19)
------------------

2.8.0 (2022-05-13)
------------------

2.7.0 (2022-04-29)
------------------

2.6.0 (2022-04-20)
------------------

2.5.0 (2022-03-25)
------------------

2.4.0 (2022-02-23)
------------------

2.3.0 (2022-02-18)
------------------

2.2.0 (2022-01-24)
------------------

2.1.0 (2022-01-11)
------------------

2.0.0 (2021-12-29)
------------------

1.2.0 (2021-11-05)
------------------
* Add verbose flag to CLI command list_controllers (`#569 <https://github.com/ros-controls/ros2_control/issues/569>`_)
* Contributors: Xi-Huang

1.1.0 (2021-10-25)
------------------
* Fixup formatting 🔧 of "list_controllers.py" and do not check formating on build stage. (`#555 <https://github.com/ros-controls/ros2_control/issues/555>`_)
  * Do not check formating on build stage.
  * Change formatting of strings.
  * Make output a bit easier to read.
* controller_manager: Use command_interface_configuration for the claimed interfaces when calling list_controllers (`#544 <https://github.com/ros-controls/ros2_control/issues/544>`_)
* Contributors: Denis Štogl, Jafar Abdi

1.0.0 (2021-09-29)
------------------
* Removed deprecated CLI verbs (`#420 <https://github.com/ros-controls/ros2_control/issues/420>`_)
* Contributors: Mathias Aarbo

0.8.0 (2021-08-28)
------------------
* fix link to point to read-the-docs (`#496 <https://github.com/ros-controls/ros2_control/issues/496>`_)
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Add index, rename cli main doc. (`#465 <https://github.com/ros-controls/ros2_control/issues/465>`_)
* fixes unload_controller issue (`#456 <https://github.com/ros-controls/ros2_control/issues/456>`_)
* Contributors: Denis Štogl, Michael, Mathias Arbo

0.7.1 (2021-06-15)
------------------

0.7.0 (2021-06-06)
------------------
* Updated arg reference to set_state from state since the argument name has been changed (`#433 <https://github.com/ros-controls/ros2_control/issues/433>`_)
* Contributors: Andrew Lycas

0.6.1 (2021-05-31)
------------------
* Use correct names after changing arguments (`#425 <https://github.com/ros-controls/ros2_control/issues/425>`_)
  In `#412 <https://github.com/ros-controls/ros2_control/issues/412>`_ we forgot to update the argument after changing flags.
* Contributors: Denis Štogl

0.6.0 (2021-05-23)
------------------
* Renaming ros2controlcli verbs (`#412 <https://github.com/ros-controls/ros2_control/issues/412>`_)
  * Renamed verbs to match services
  * README.rst redirects to docs/index.rst
  * argument {start/stop}_controllers -> {start/stop}
  * rst include did not work, try relative link
  * Moved configure_controller doc to deprecated
  * set_state -> set-state
* Contributors: Mathias Hauan Arbo, Denis Štogl

0.5.0 (2021-05-03)
------------------
* correct return values in CLI (`#401 <https://github.com/ros-controls/ros2_control/issues/401>`_)
* [python] Update files in ros2controlcli to use format strings (`#358 <https://github.com/ros-controls/ros2_control/issues/358>`_)
* Add starting doc for ros2controlcli (`#377 <https://github.com/ros-controls/ros2_control/issues/377>`_)
* Contributors: Bence Magyar, Karsten Knese, NovusEdge

0.4.0 (2021-04-07)
------------------
* Remodel ros2controlcli, refactor spawner/unspawner and fix test (`#349 <https://github.com/ros-controls/ros2_control/issues/349>`_)
* Contributors: Karsten Knese

0.3.0 (2021-03-21)
------------------

0.2.1 (2021-03-02)
------------------

0.2.0 (2021-02-26)
------------------
* Increase service call timeout, often services take longer than 0.2s (`#324 <https://github.com/ros-controls/ros2_control/issues/324>`_)
* Contributors: Victor Lopez

0.1.6 (2021-02-05)
------------------

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------
* Print error messages if ros2controlcli commands fail (`#309 <https://github.com/ros-controls/ros2_control/issues/309>`_)
* Inverse the response of cli commands to return correct exit-status. (`#308 <https://github.com/ros-controls/ros2_control/issues/308>`_)
  * Inverse the response of cli commands to return correct exit-status.
  * list verbs return exit-status 0
* Contributors: Shota Aoki, Victor Lopez

0.1.3 (2021-01-21)
------------------

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
* Add configure controller service (`#272 <https://github.com/ros-controls/ros2_control/issues/272>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* Add cli interface (`#176 <https://github.com/ros-controls/ros2_control/issues/176>`_)
* Contributors: Bence Magyar, Denis Štogl, Karsten Knese, Victor Lopez
