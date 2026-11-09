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

import array

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.events import Shutdown

from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import Parameter
from launch_ros.utilities.dump_params import apply_node_remaps_to_fqn
from launch_ros.utilities.dump_params import attach_dump_params_collector
from launch_ros.utilities.dump_params import DumpParamsCollector
from launch_ros.utilities.dump_params import DumpParamsError

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


def _dump_params(actions):
    collector = DumpParamsCollector()
    ls = LaunchService(noninteractive=True)
    attach_dump_params_collector(ls.context, collector)
    ls.include_launch_description(LaunchDescription([
        *actions,
        EmitEvent(event=Shutdown(reason='test')),
    ]))
    assert ls.run() == 0
    return yaml.safe_load(collector.dumps())


def test_dump_params_same_name_later_source_wins():
    # Runtime argv order (not launch-file order): ros_arguments, then
    # SetParameter, then parameters=[...]. Later entry wins, matching rcl.
    dumped = _dump_params([
        SetParameter(name='from_global_and_ros', value='from_global'),
        SetParameter(name='from_all_three', value='from_global'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/',
            parameters=[{
                'from_all_three': 'from_parameters',
                'from_parameters_and_ros': 'from_parameters',
                'only_parameters': 1,
            }],
            ros_arguments=[
                '-p', 'from_global_and_ros:=from_ros',
                '-p', 'from_all_three:=from_ros',
                '-p', 'from_parameters_and_ros:=from_ros',
                '-p', 'only_ros:=2',
            ],
        ),
    ])
    params = dumped['/talker']['ros__parameters']
    assert params['from_global_and_ros'] == 'from_global'
    assert params['from_all_three'] == 'from_parameters'
    assert params['from_parameters_and_ros'] == 'from_parameters'
    assert params['only_parameters'] == 1
    assert params['only_ros'] == 2


def test_dump_params_parameter_descriptions():
    dumped = _dump_params([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/',
            parameters=[
                Parameter('x', 1),
                Parameter('labels', ['a', 'b']),
            ],
            ros_arguments=['-p', 'x:=99'],
        ),
    ])
    params = dumped['/talker']['ros__parameters']
    assert params['x'] == 1
    assert params['labels'] == ['a', 'b']


def test_dump_params_composable_node_arrays():
    dumped = _dump_params([
        LoadComposableNodes(
            target_container='my_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp',
                    plugin='demo_nodes_cpp::Talker',
                    name='talker',
                    namespace='demo',
                    parameters=[{
                        'ids': [1, 2, 3],
                        'gains': [0.5, 1.5],
                        'enabled': True,
                    }],
                ),
            ],
        ),
    ])
    params = dumped['/demo/talker']['ros__parameters']
    assert params['ids'] == [1, 2, 3]
    assert params['gains'] == [0.5, 1.5]
    assert params['enabled'] is True


def test_collector_dumps_array_values():
    collector = DumpParamsCollector()
    collector.add_node('/n', {
        'ids': array.array('q', [1, 2, 3]),
        'gains': array.array('d', [0.5, 1.5]),
    })
    dumped = yaml.safe_load(collector.dumps())
    params = dumped['/n']['ros__parameters']
    assert params['ids'] == [1, 2, 3]
    assert params['gains'] == [0.5, 1.5]
