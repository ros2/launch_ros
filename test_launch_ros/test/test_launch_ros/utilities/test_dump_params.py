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

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.utilities.dump_params import apply_node_remaps_to_fqn
from launch_ros.utilities.dump_params import attach_dump_params_collector
from launch_ros.utilities.dump_params import DumpParamsCollector
from launch_ros.utilities.dump_params import DumpParamsError

import pytest


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


def test_dump_params_node_set_parameter_and_namespace():
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
