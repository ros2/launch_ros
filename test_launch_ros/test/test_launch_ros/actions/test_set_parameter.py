# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetParameter Action."""

from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode

import pytest
import yaml


class MockContext:

    def __init__(self):
        self.launch_configurations = {}

    def perform_substitution(self, sub):
        return sub.perform(None)


def get_set_parameter_test_parameters():
    return [
        pytest.param(
            [{'my_param': '2'}],  # to set
            {'my_param': '2'},  # expected
            id='One param'
        ),
        pytest.param(
            [{'my_param': '2', 'another_param': '2'}, {'my_param': '3'}],
            {'my_param': '3', 'another_param': '2'},
            id='Two params, overriding one'
        ),
    ]


@pytest.mark.parametrize(
    'params_to_set, expected_result',
    get_set_parameter_test_parameters()
)
def test_set_param(params_to_set, expected_result):
    set_parameter_actions = []
    for dicts in params_to_set:
        for name, value in dicts.items():
            set_parameter_actions.append(SetParameter(name=name, value=value))
    lc = MockContext()
    for set_param in set_parameter_actions:
        set_param.execute(lc)
    assert lc.launch_configurations == {'ros_params': expected_result}


def test_set_param_with_node():
    lc = MockContext()
    node = Node(
        package='asd',
        executable='bsd',
        name='my_node',
        namespace='my_ns',
        parameters=[{'asd': 'bsd'}]
    )
    set_param = SetParameter(name='my_param', value='my_value')
    set_param.execute(lc)
    node._perform_substitutions(lc)
    expanded_parameter_files = node._Node__expanded_parameter_files
    assert len(expanded_parameter_files) == 2
    with open(expanded_parameter_files[0], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == {
            '/my_ns/my_node': {
                'ros__parameters': {'my_param': 'my_value'}
            }
        }
    with open(expanded_parameter_files[1], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == {
            '/my_ns/my_node': {
                'ros__parameters': {'asd': 'bsd'}
            }
        }


def test_set_param_with_composable_node():
    lc = MockContext()
    node_description = ComposableNode(
        package='asd',
        plugin='my_plugin',
        name='my_node',
        namespace='my_ns',
        parameters=[{'asd': 'bsd'}]
    )
    set_param_1 = SetParameter(name='my_param', value='my_value')
    set_param_2 = SetParameter(name='asd', value='csd')
    set_param_1.execute(lc)
    set_param_2.execute(lc)
    request = get_composable_node_load_request(node_description, lc)
    parameters = request.parameters
    assert len(parameters) == 3
    assert parameters[0].name == 'my_param'
    assert parameters[0].value.string_value == 'my_value'
    assert parameters[1].name == 'asd'
    assert parameters[1].value.string_value == 'csd'
    assert parameters[2].name == 'asd'
    assert parameters[2].value.string_value == 'bsd'

    lc = MockContext()
    node_description = ComposableNode(
        package='asd',
        plugin='my_plugin',
        name='my_node',
        namespace='my_ns',
    )
    set_param = SetParameter(name='my_param', value='my_value')
    set_param.execute(lc)
    request = get_composable_node_load_request(node_description, lc)
    parameters = request.parameters
    assert len(parameters) == 1
    assert parameters[0].name == 'my_param'
    assert parameters[0].value.string_value == 'my_value'
