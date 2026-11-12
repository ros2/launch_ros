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

"""Tests for launch_ros.descriptions.ParameterFile."""

from contextlib import contextmanager
import os
from tempfile import NamedTemporaryFile

from launch import LaunchContext
from launch import Substitution
from launch.conditions import IfCondition
from launch.frontend import expose_substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.utilities import to_parameters_list

import pytest
import rclpy
import yaml


@contextmanager
def get_parameter_file(contents, mode='w'):
    f = NamedTemporaryFile(delete=False, mode=mode)
    try:
        with f:
            f.write(contents)
        yield f.name
    finally:
        os.unlink(f.name)


class MockContext:

    def perform_substitution(self, sub):
        return sub.perform(None)


class CustomSubstitution(Substitution):

    def __init__(self, text):
        self.__text = text

    def perform(self, context):
        return self.__text


@expose_substitution('test')
def parse_test_substitution(data):
    if not data or len(data) > 1:
        raise RuntimeError()
    kwargs = {}
    kwargs['text'] = perform_substitutions(MockContext(), data[0])
    return CustomSubstitution, kwargs


def get_test_cases():
    parameter_file_without_substitutions = (
        """\
/my_ns/my_node:
    ros__parameters:
        my_int: 1
        my_str: '1'
        my_list_of_strs: ['1', '2', '3']
        """
    )
    parameter_file_with_substitutions = (
        """\
'/$(test my_ns)/$(test my_node)':
    ros__parameters:
        my_int: '$(test 1)'
        my_str: '"$(test 1)"'
        my_list_of_strs: ['1', '"$(test 2)"', '3']
        """
    )
    substituted_file = (
        """\
'/my_ns/my_node':
    ros__parameters:
        my_int: '1'
        my_str: '"1"'
        my_list_of_strs: ['1', '"2"', '3']
        """
    )
    return [
        pytest.param(
            parameter_file_with_substitutions,  # original contents
            substituted_file,  # expected contents
            True,  # substitutions allowed
            id='Parameter file with substitutions, substitutions allowed',
        ),
        pytest.param(
            parameter_file_with_substitutions,
            parameter_file_with_substitutions,
            False,
            id='Parameter file with substitutions, substitutions not allowed',
        ),
        pytest.param(
            parameter_file_without_substitutions,
            parameter_file_without_substitutions,
            True,
            id='Parameter file without substitutions, substitutions allowed',
        ),
        pytest.param(
            parameter_file_without_substitutions,
            parameter_file_without_substitutions,
            False,
            id='Parameter file without substitutions, substitutions not allowed',
        ),
    ]


@pytest.mark.parametrize(
    'original_contents, expected_contents, allow_substs',
    get_test_cases(),
)
def test_parameter_file_description(original_contents, expected_contents, allow_substs):
    lc = MockContext()
    with get_parameter_file(original_contents) as file_name:
        desc = ParameterFile(file_name, allow_substs=allow_substs)
        if isinstance(desc.param_file, list):
            assert perform_substitutions(lc, desc.param_file) == file_name
        else:
            assert desc.param_file == file_name
        assert desc.allow_substs == allow_substs
        evaluated_param_file = desc.evaluate(lc)
        with open(evaluated_param_file, 'r') as new_f:
            assert new_f.read() == expected_contents
        assert desc.param_file == evaluated_param_file
        if not allow_substs:
            assert os.fspath(desc.param_file) == os.fspath(file_name)
        param_file = desc.param_file
        desc.cleanup()
        if allow_substs:
            assert not param_file.exists()
            assert isinstance(desc.param_file, list)
        else:
            assert param_file.exists()
            assert os.fspath(desc.param_file) == os.fspath(file_name)


@pytest.mark.parametrize(
    'namespace, use_namespace, expected_namespace',
    [
        ('', 'true', '/'),
        ('/', 'true', '/'),
        ('my_ns', 'true', '/my_ns'),
        ('/my_ns', 'true', '/my_ns'),
        ('my_ns/', 'true', '/my_ns'),
        ('my_ns/nested', 'true', '/my_ns/nested'),
        ('my_ns', 'false', '/'),
    ],
)
def test_parameter_file_with_optional_namespace(namespace, use_namespace, expected_namespace):
    context = LaunchContext()
    context.launch_configurations['namespace'] = namespace
    PushROSNamespace(
        LaunchConfiguration('namespace'),
        condition=IfCondition(use_namespace),
    ).visit(context)
    contents = """\
$(eval "'$(var ros_namespace /)'.rstrip('/')")/my_node:
    ros__parameters:
        my_int: 42
        my_float: 1.25
        my_bool: true
        my_str: '42'
        my_list: [1, 2, 3]
"""
    expected_values = {
        'my_int': 42,
        'my_float': 1.25,
        'my_bool': True,
        'my_str': '42',
        'my_list': [1, 2, 3],
    }
    with get_parameter_file(contents) as file_name:
        desc = ParameterFile(file_name, allow_substs=True)
        try:
            param_file = desc.evaluate(context)
            expected_name = expected_namespace.rstrip('/') + '/my_node'
            with open(param_file, 'r') as f:
                assert yaml.safe_load(f) == {
                    expected_name: {'ros__parameters': expected_values},
                }

            parameters = to_parameters_list(context, 'my_node', expected_namespace, [param_file])
            values = {}
            for parameter in parameters:
                value = parameter.value
                if parameter.type_ == rclpy.Parameter.Type.INTEGER_ARRAY:
                    value = list(value)
                values[parameter.name] = value
            assert values == expected_values

            ros_context = rclpy.context.Context()
            with rclpy.init(
                args=['--ros-args', '--params-file', str(param_file)],
                context=ros_context,
            ):
                node = rclpy.create_node(
                    'my_node',
                    namespace=expected_namespace,
                    context=ros_context,
                    automatically_declare_parameters_from_overrides=True,
                )
                try:
                    assert node.get_fully_qualified_name() == expected_name
                    for name, value in expected_values.items():
                        parameter = node.get_parameter(name)
                        if isinstance(value, list):
                            assert parameter.type_ == rclpy.Parameter.Type.INTEGER_ARRAY
                            assert list(parameter.value) == value
                        else:
                            assert parameter.value == value
                            assert type(parameter.value) is type(value)
                finally:
                    node.destroy_node()
        finally:
            desc.cleanup()
        assert not param_file.exists()
