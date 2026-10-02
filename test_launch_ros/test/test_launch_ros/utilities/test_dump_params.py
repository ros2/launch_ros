# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Tests for dump-params utilities and dry-run collection."""

import io
from unittest.mock import patch

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.utilities.dump_params import DumpParamsCollector
from launch_ros.utilities.dump_params import DumpParamsError
from launch_ros.utilities.dump_params import apply_node_remaps_to_fqn
from launch_ros.utilities.dump_params import attach_dump_params_collector

import pytest
import yaml


def test_apply_node_remaps_to_fqn():
    assert apply_node_remaps_to_fqn('/nav/amcl', None) == '/nav/amcl'
    assert apply_node_remaps_to_fqn(
        '/nav/amcl', [('__node', 'amcl2')]) == '/nav/amcl2'
    assert apply_node_remaps_to_fqn(
        '/nav/amcl', [('__ns', '/other')]) == '/other/amcl'


def test_collector_coalesce_and_diverge():
    collector = DumpParamsCollector()
    collector.add_node('/a', {'x': 1})
    collector.add_node('/a', {'x': 1})  # identical → coalesce
    with pytest.raises(DumpParamsError):
        collector.add_node('/a', {'x': 2})


def test_dump_params_node_set_parameter_and_namespace(tmp_path):
    collector = DumpParamsCollector()
    ls = LaunchService(noninteractive=True)
    attach_dump_params_collector(ls.context, collector)

    ld = LaunchDescription([
        GroupAction([
            PushRosNamespace('nav'),
            SetParameter(name='use_sim_time', value=True),
            Node(
                package='demo_nodes_cpp',
                executable='talker',
                name='amcl',
                parameters=[{
                    'max_particles': 2000,
                    'robot_model_type': 'differential',
                }],
            ),
        ]),
        EmitEvent(event=Shutdown(reason='test')),
    ])
    ls.include_launch_description(ld)
    assert ls.run() == 0

    dumped = collector.to_yaml_dict()
    assert '/nav/amcl' in dumped
    params = dumped['/nav/amcl']['ros__parameters']
    assert params['use_sim_time'] is True
    assert params['max_particles'] == 2000
    assert params['robot_model_type'] == 'differential'


def test_dump_params_of_a_launch_file_api(tmp_path):
    launch_path = tmp_path / 'dump_params_test.launch.py'
    launch_path.write_text(
        """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='demo',
            parameters=[{'foo': 1, 'bar': 'baz'}],
            ros_arguments=['-p', 'extra:=2'],
        ),
    ])
"""
    )

    from ros2launch.api import dump_params_of_a_launch_file

    buf = io.StringIO()
    with patch('sys.stdout', buf):
        rc = dump_params_of_a_launch_file(launch_file_path=str(launch_path))
    assert rc == 0
    data = yaml.safe_load(buf.getvalue())
    assert '/demo/talker' in data
    params = data['/demo/talker']['ros__parameters']
    assert params['foo'] == 1
    assert params['bar'] == 'baz'
    assert params['use_sim_time'] is True
    assert params['extra'] == 2
