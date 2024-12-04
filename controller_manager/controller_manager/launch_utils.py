# Copyright (c) 2021 PAL Robotics S.L.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_controllers_spawner_launch_description(
    controller_names: list, controller_params_files=None, extra_spawner_args=[]
):
    """
    Generate launch description for loading a controller using spawner.

    Returns a list of LaunchDescription actions adding the 'controller_manager_name' and
    'unload_on_kill' LaunchArguments and a Node action that runs the controller_manager
    spawner node to load and activate a controller

    Examples
    --------
      # Assuming the controller parameters are known to the controller_manager
      generate_controllers_spawner_launch_description(['joint_state_broadcaster'])

      # Passing controller parameter file to load the controller (Controller type is retrieved from config file)
      generate_controllers_spawner_launch_description(
        ['joint_state_broadcaster'],
        controller_params_files=[os.path.join(get_package_share_directory('my_pkg'),
                                            'config', 'controller_params.yaml')],
        extra_spawner_args=[--load-only]
        )

    """
    declare_controller_mgr_name = DeclareLaunchArgument(
        "controller_manager_name",
        default_value="controller_manager",
        description="Controller manager node name",
    )
    declare_unload_on_kill = DeclareLaunchArgument(
        "unload_on_kill",
        default_value="false",
        description="Wait until the node is interrupted and then unload controller",
    )

    spawner_arguments = controller_names
    spawner_arguments.extend(
        [
            "--controller-manager",
            LaunchConfiguration("controller_manager_name"),
        ]
    )

    if controller_params_files:
        for controller_params_file in controller_params_files:
            if controller_params_file:
                spawner_arguments += ["--param-file", controller_params_file]

    # Setting --unload-on-kill if launch arg unload_on_kill is "true"
    # See https://github.com/ros2/launch/issues/290
    spawner_arguments += [
        PythonExpression(
            [
                '"--unload-on-kill"',
                ' if "true" == "',
                LaunchConfiguration("unload_on_kill"),
                '" else ""',
            ]
        )
    ]

    if extra_spawner_args:
        spawner_arguments += extra_spawner_args

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=spawner_arguments,
        shell=True,
        output="screen",
    )

    return LaunchDescription(
        [
            declare_controller_mgr_name,
            declare_unload_on_kill,
            spawner,
        ]
    )


def generate_controllers_spawner_launch_description_from_dict(
    controller_info_dict: dict, extra_spawner_args=[]
):
    """
    Generate launch description for loading a controller using spawner.

    controller_info_dict: dict
        A dictionary with the following info:
        - controller_name: str
            The name of the controller to load as the key
        - controller_params_file: str or list or None
            The path to the controller parameter file or a list of paths to multiple parameter files
            or None if no parameter file is needed as the value of the key
            If a list is passed, the controller parameters will be overloaded in same order
    extra_spawner_args: list
        A list of extra arguments to pass to the controller spawner
    """
    if not type(controller_info_dict) is dict:
        raise ValueError(f"Invalid controller_info_dict type parsed {controller_info_dict}")
    controller_names = controller_info_dict.keys()
    controller_params_files = []
    for controller_name in controller_names:
        controller_params_file = controller_info_dict[controller_name]
        if controller_params_file:
            if type(controller_params_file) is list:
                controller_params_files.extend(controller_params_file)
            elif type(controller_params_file) is str:
                controller_params_files.append(controller_params_file)
            else:
                raise ValueError(
                    f"Invalid controller_params_file type parsed in the dict {controller_params_file}"
                )
    return generate_controllers_spawner_launch_description(
        controller_names=controller_names,
        controller_params_files=controller_params_files,
        extra_spawner_args=extra_spawner_args,
    )


def generate_load_controller_launch_description(
    controller_name: str, controller_params_file=None, extra_spawner_args=[]
):
    controller_params_files = [controller_params_file] if controller_params_file else None
    return generate_controllers_spawner_launch_description(
        controller_names=[controller_name],
        controller_params_files=controller_params_files,
        extra_spawner_args=extra_spawner_args,
    )
