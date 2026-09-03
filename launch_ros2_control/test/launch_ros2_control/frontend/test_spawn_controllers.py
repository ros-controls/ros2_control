# Copyright 2025 Jasper van Brakel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import io
import textwrap

from launch import LaunchService
from launch.condition import Condition
from launch.frontend import Parser
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters
import osrf_pycommon.process_utils


def test_launch_spawn_controllers_yaml():
    yaml_file = textwrap.dedent(r"""
        launch:
            - spawn_controller:
                controller_manager: /my/controller_manager
                unload_on_kill: true
                activate_as_group: true
                controller_manager_timeout: 9.
                switch_timeout: 10.
                service_call_timeout: 10.
                controller:
                    -   name: my_controller
                        remap:
                            -   from: me
                                to: /you
                        param:
                            - name: update_rate
                              value: 9
                    -   name: second_controller
                        if: True
                        remap:
                            -   from: something
                                to: else
        """)
    with io.StringIO(yaml_file) as f:
        check_launch_spawn_controllers(f)


def test_launch_spawn_controllers_xml():
    xml_file = textwrap.dedent(r"""
        <launch>
            <spawn_controller controller_manager="/my/controller_manager" unload_on_kill="True"
                activate_as_group="true" controller_manager_timeout="9" switch_timeout="10" service_call_timeout="10">
                <controller name="my_controller">
                    <remap from="me" to="/you" />
                    <param name="update_rate" value="9" />
                </controller>
                <controller name="second_controller" if="True">
                    <remap from="something" to="else" />
                </controller>
            </spawn_controller>
        </launch>
        """)  # noqa: E501
    with io.StringIO(xml_file) as f:
        check_launch_spawn_controllers(f)


def check_launch_spawn_controllers(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)

    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())

    (controller_spawner,) = ld.describe_sub_entities()
    my_controller = controller_spawner._SpawnControllers__controller_descriptions[0]
    second_controller = controller_spawner._SpawnControllers__controller_descriptions[1]

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    def perform_if(substitution, data_type):
        if isinstance(substitution, data_type):
            return substitution
        return perform(substitution)

    # TODO: Check Controller Spawner params
    assert (
        perform(controller_spawner._SpawnControllers__controller_manager)
        == "/my/controller_manager"
    )

    assert perform_if(controller_spawner._SpawnControllers__unload_on_kill, bool)
    assert perform_if(controller_spawner._SpawnControllers__activate_as_group, bool)

    assert (
        perform_if(controller_spawner._SpawnControllers__controller_manager_timeout, float) == 9.0
    )
    assert perform_if(controller_spawner._SpawnControllers__switch_timeout, float) == 10.0
    assert perform_if(controller_spawner._SpawnControllers__service_call_timeout, float) == 10.0

    # Check Controller parameters
    my_controller_remappings = list(my_controller.remappings)
    second_controller_remappings = list(second_controller.remappings)

    my_controller_parameters = evaluate_parameters(ls.context, my_controller.parameters)

    assert perform(my_controller.controller_name) == "my_controller"
    assert my_controller.condition is None
    assert len(my_controller_remappings) == 1
    assert (perform(my_controller_remappings[0][0]), perform(my_controller_remappings[0][1])) == (
        "me",
        "/you",
    )
    assert len(my_controller_parameters) == 1
    assert my_controller_parameters[0].get("update_rate") == 9

    assert perform(second_controller.controller_name) == "second_controller"
    assert isinstance(second_controller.condition, Condition)
    assert second_controller.condition.evaluate(ls.context)
    assert (
        perform(second_controller_remappings[0][0]),
        perform(second_controller_remappings[0][1]),
    ) == ("something", "else")

    # TODO: This executes the spawner, but the spawner will not actually run,
    #       since there is no controller manager.
    timeout_sec = 5
    loop.run_until_complete(asyncio.sleep(timeout_sec))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    assert 0 == launch_task.result()
