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
            [{'my_param': '2'}],  # expected
            id='One param'
        ),
        pytest.param(
            [{'my_param': '2', 'another_param': '2'}, {'my_param': '3'}],
            [{'my_param': '3', 'another_param': '2'}],
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
    lc.launch_configurations == {'ros_params': expected_result}


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
