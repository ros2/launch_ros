# Copyright 2026 ROS 2 contributors
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

"""
Load one parameter file with or without a ROS namespace.

From the repository root in a sourced ROS environment with demo_nodes_cpp installed:

    ros2 launch launch_ros/examples/optional_namespace.launch.py

In another sourced terminal, check that some_int is 42:

    ros2 param get /parameter_blackboard some_int

Stop the launch, then run it with a namespace and query the same parameter:

    ros2 launch launch_ros/examples/optional_namespace.launch.py namespace:=robot1
    ros2 param get /robot1/parameter_blackboard some_int

ROBOT_NAMESPACE supplies the default namespace. Passing use_namespace:=false
skips the namespace push even when that environment variable is set.
The YAML selector uses the effective ros_namespace and strips its trailing slash
before adding the node name, so the root namespace does not produce a double slash.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    """Launch a parameter blackboard with an optional namespace."""
    params = Path(__file__).with_name('optional_namespace.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=EnvironmentVariable('ROBOT_NAMESPACE', default_value=''),
            description='Namespace to use when use_namespace is true.',
        ),
        DeclareLaunchArgument('use_namespace', default_value='true'),
        GroupAction([
            PushROSNamespace(
                LaunchConfiguration('namespace'),
                condition=IfCondition(LaunchConfiguration('use_namespace')),
            ),
            Node(
                package='demo_nodes_cpp',
                executable='parameter_blackboard',
                name='parameter_blackboard',
                parameters=[ParameterFile(params, allow_substs=True)],
                output='screen',
            ),
        ]),
    ])
