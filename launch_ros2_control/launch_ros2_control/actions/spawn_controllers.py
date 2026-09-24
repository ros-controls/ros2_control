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

"""Module for the spawn_controller action."""

import itertools
import os
from pathlib import Path
from typing import cast, List, Optional, TYPE_CHECKING, Union

import launch
from launch.action import Action
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter, ParameterFile
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict

if TYPE_CHECKING:
    from launch_ros.parameters_type import Parameters

from ..descriptions import Controller

FloatSubstitution = Union[List[Substitution], float]
# TODO: Possibly add Controller or Node as subelement when printing


@expose_action("spawn_controller")
@expose_action("spawn-controller")
class SpawnControllers(Action):
    def __init__(
        self,
        controller_descriptions: List[Controller],
        *,
        spawner_name: Optional[SomeSubstitutionsType] = None,
        controller_manager: Optional[SomeSubstitutionsType] = None,
        # controller_remappings: Optional[SomeRemapRules] = None,
        final_controller_state: SomeSubstitutionsType = "active",
        unload_on_kill: Union[bool, SomeSubstitutionsType] = False,
        activate_as_group: Union[bool, SomeSubstitutionsType] = False,
        # FLAG controller_ros_args (maybe omit)
        controller_manager_timeout: Optional[Union[float, SomeSubstitutionsType]] = None,
        switch_timeout: Optional[Union[float, SomeSubstitutionsType]] = None,
        service_call_timeout: Optional[Union[float, SomeSubstitutionsType]] = None,
        emulate_tty: Union[bool, SomeSubstitutionsType] = False,
        output: SomeSubstitutionsType = "log",
        **kwargs,
    ) -> None:
        """
        Construct a SpawnControllers.

        :param: controller_descriptions descriptions of controllers to be spawned
        :param: spawner_name the spawner node name
        :param: controller_manager the controller manager node spawn the controllers into
        :param: final_controller_state the state the controllers should end in, defaults to
            'active', but can also be 'inactive' or 'unconfigured'. ('active' adds no extra flag,
            'inactive' adds '--inactive' and 'unconfigured' adds '--load-only')
            Throws :py:exc:`ValueError` if not either 'active', 'inactive' or 'unconfigured'.
        :param: unload_on_kill if 'True' unload the controller if the process is stopped.
            Either a boolean or a Substitution to be resolved at runtime. Defaults to 'False'.
        :param: activate_as_group if 'True' activate all provided controllers at once.
            Either a boolean or a Substitution to be resolved at runtime. Defaults to 'False'.
        :param: controller_manager_timeout time to wait for the controller manager
        :param: switch_timeout time to wait for a successful state switch of the controllers
        :param: service_call_timeout time to wait for service responses of the controller manager
        :param: emulate_tty emulate a tty (terminal), defaults to False, but can
            be overridden with the LaunchConfiguration called 'emulate_tty',
            the value of which is evaluated as true or false according to
            :py:func:`evaluate_condition_expression`.
            Throws :py:exc:`InvalidConditionExpressionError` if the
            'emulate_tty' configuration does not represent a boolean.
        :param: output configuration for process output logging. Defaults to 'log'
            i.e. log both stdout and stderr to launch main log file and stderr to
            the screen.
            Overridden externally by the OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar value.
            See `launch.logging.get_output_loggers()` documentation for further
            reference on all available options.
        """
        super().__init__(**kwargs)

        # NOTE(SuperJappie08): Only have to normalize attributes, which are used here.
        #                      If something is only passed, it will be normalized by Node.

        self.__controller_descriptions = controller_descriptions
        self.__spawner_name = spawner_name

        self.__controller_manager = None  # type: Optional[List[Substitution]]
        if controller_manager is not None:
            self.__controller_manager = normalize_to_list_of_substitutions(controller_manager)

        self.__final_controller_state = normalize_to_list_of_substitutions(final_controller_state)

        self.__unload_on_kill = normalize_typed_substitution(unload_on_kill, bool)

        self.__activate_as_group = normalize_typed_substitution(activate_as_group, bool)

        self.__controller_manager_timeout: "Optional[FloatSubstitution]" = None
        if controller_manager_timeout is not None:
            self.__controller_manager_timeout = cast(
                FloatSubstitution, normalize_typed_substitution(controller_manager_timeout, float)
            )

        self.__switch_timeout = None  # type: Optional[FloatSubstitution]
        if switch_timeout is not None:
            self.__switch_timeout = cast(
                FloatSubstitution, normalize_typed_substitution(switch_timeout, float)
            )

        self.__service_call_timeout = None  # type: Optional[FloatSubstitution]
        if service_call_timeout is not None:
            self.__service_call_timeout = cast(
                FloatSubstitution, normalize_typed_substitution(service_call_timeout, float)
            )

        self.__emulate_tty = normalize_typed_substitution(emulate_tty, bool)
        self.__output = output

        self.__logger = launch.logging.get_logger(__name__)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse spawn_controller."""
        _, kwargs = super().parse(entity, parser)

        spawner_name = entity.get_attr("spawner_name", data_type=str, optional=True)
        if spawner_name is not None:
            kwargs["spawner_name"] = parser.parse_substitution(spawner_name)

        controller_manager = entity.get_attr("controller_manager", data_type=str, optional=True)
        if controller_manager is not None:
            kwargs["controller_manager"] = parser.parse_substitution(controller_manager)

        final_controller_state = entity.get_attr(
            "final_controller_state", data_type=str, optional=True
        )
        if final_controller_state is not None:
            kwargs["final_controller_state"] = parser.parse_substitution(final_controller_state)

        unload_on_kill = entity.get_attr("unload_on_kill", data_type=bool, optional=True)
        if unload_on_kill is not None:
            kwargs["unload_on_kill"] = parser.parse_if_substitutions(unload_on_kill)

        activate_as_group = entity.get_attr("activate_as_group", data_type=bool, optional=True)
        if activate_as_group is not None:
            kwargs["activate_as_group"] = parser.parse_if_substitutions(activate_as_group)

        controller_manager_timeout = entity.get_attr(
            "controller_manager_timeout", data_type=float, optional=True
        )
        if controller_manager_timeout is not None:
            kwargs["controller_manager_timeout"] = parser.parse_if_substitutions(
                controller_manager_timeout
            )

        switch_timeout = entity.get_attr("switch_timeout", data_type=float, optional=True)
        if switch_timeout is not None:
            kwargs["switch_timeout"] = parser.parse_if_substitutions(switch_timeout)

        service_call_timeout = entity.get_attr(
            "service_call_timeout", data_type=float, optional=True
        )
        if service_call_timeout is not None:
            kwargs["service_call_timeout"] = parser.parse_if_substitutions(service_call_timeout)

        controllers = entity.get_attr("controller", data_type=List[Entity])
        kwargs["controller_descriptions"] = []
        for controller in controllers:
            controller_cls, controller_kwargs = Controller.parse(parser, controller)
            kwargs["controller_descriptions"].append(controller_cls(**controller_kwargs))

        emulate_tty = entity.get_attr("emulate_tty", data_type=bool, optional=True)
        if emulate_tty is not None:
            kwargs["emulate_tty"] = parser.parse_if_substitutions(emulate_tty)

        output = entity.get_attr("output", optional=True)
        if output is not None:
            kwargs["output"] = parser.parse_substitution(output)

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        launch_descriptions: List[Action] = []

        controllers: List[SomeSubstitutionsType] = []
        other_arguments: List[SomeSubstitutionsType] = []

        if self.__controller_manager is not None:
            other_arguments += ["--controller-manager", self.__controller_manager]

        final_controller_state = perform_substitutions(
            context, self.__final_controller_state
        ).lower()
        if final_controller_state == "inactive":
            other_arguments.append("--inactive")
        elif final_controller_state == "unconfigured":
            other_arguments.append("--load-only")
        elif final_controller_state != "active":
            raise ValueError(
                "invalid final controller state received '{}' "
                "[Expected one of ('active', 'inactive', 'unconfigured')]".format(
                    final_controller_state
                )
            )

        unload_on_kill = cast(
            bool, perform_typed_substitution(context, self.__unload_on_kill, bool)
        )
        if unload_on_kill:
            other_arguments.append("--unload-on-kill")

        activate_as_group = cast(
            bool, perform_typed_substitution(context, self.__activate_as_group, bool)
        )
        if activate_as_group:
            other_arguments.append("--activate-as-group")

        if (timeout_sub := self.__controller_manager_timeout) is not None:
            timeout = cast(float, perform_typed_substitution(context, timeout_sub, float))
            other_arguments += ["--controller-manager-timeout", str(timeout)]

        if (timeout_sub := self.__switch_timeout) is not None:
            timeout = cast(float, perform_typed_substitution(context, timeout_sub, float))
            other_arguments += ["--switch-timeout", str(timeout)]

        if (timeout_sub := self.__service_call_timeout) is not None:
            timeout = cast(float, perform_typed_substitution(context, timeout_sub, float))
            other_arguments += ["--service-call-timeout", str(timeout)]

        # Parse global params
        params_container = context.launch_configurations.get("global_params", None)
        extra_params: "Optional[Parameters]" = None

        if params_container is not None:

            def convert_param(param):
                if isinstance(param, tuple):
                    return normalize_parameter_dict({param[0]: param[1]})
                else:
                    param_file_path = Path(param).resolve()
                    assert param_file_path.is_file()
                    return ParameterFile(param_file_path)

            extra_params = list(map(convert_param, params_container))

        # Parse global remaps
        global_remaps = context.launch_configurations.get("ros_remaps", None)
        extra_remaps = []

        if global_remaps is not None:
            extra_remaps += global_remaps

        for controller in self.__controller_descriptions:
            if controller.condition is not None and not controller.condition.evaluate(context):
                continue

            controllers.append(controller.controller_name)

            combined_parameters = list(extra_params) if extra_params is not None else []
            if controller.parameters:
                combined_parameters += controller.parameters

            if combined_parameters:
                evaluated_parameters = evaluate_parameters(context, combined_parameters)

                # Load normally for dict and path/file, since no combined file needs to be made
                # TODO(SuperJappie08): It would be nice to combine continues sections of Parameters
                #                      into a single file, since it would prevent a lot of files.
                for params in evaluated_parameters:
                    if isinstance(params, dict):
                        params_argument = controller._create_params_file_from_dict(context, params)
                        assert os.path.isfile(params_argument)
                    elif isinstance(params, Path):
                        params_argument = str(params)
                    elif isinstance(params, Parameter):
                        # NOTE: This only occurs, if somebody explicitly passes a Parameter object
                        # FIXME(SuperJappie08): This makes a lot of temporary files, could combine
                        #                       the values if present (and non-interrupted).
                        name, value = params.evaluate(context)
                        params_argument = controller._create_params_file_from_dict(
                            context, {name: value}
                        )
                        assert os.path.isfile(params_argument)
                    else:
                        raise RuntimeError(f"invalid normalized parameters {repr(params)}")

                    if not os.path.isfile(params_argument):
                        self.__logger.warning(
                            f"Parameter file path is not a file: {params_argument}"
                        )
                        continue
                    other_arguments += ["-p", params_argument]

            combined_remappings = extra_remaps.copy()
            if controller.remappings:
                combined_remappings += list(controller.remappings)

            if combined_remappings:
                for src, dst in combined_remappings:
                    other_arguments += [
                        "--controller-ros-args",
                        itertools.chain.from_iterable(
                            (
                                "-r ",
                                controller.controller_name,
                                ":",
                                src,
                                ":=",
                                dst,
                            )
                        ),
                    ]

        launch_descriptions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                name=self.__spawner_name,
                arguments=controllers + other_arguments,
                output=self.__output,
                emulate_tty=self.__emulate_tty,
            )
        )

        return launch_descriptions
