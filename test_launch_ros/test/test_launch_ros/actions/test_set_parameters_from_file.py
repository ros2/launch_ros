# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetParametersFromFile Action."""

import os
import tempfile

from launch import LaunchContext
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations
from launch.substitutions import SubstitutionFailure
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions

from launch_ros.actions import Node
from launch_ros.actions import SetParameter, SetParametersFromFile
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode

import pytest
import yaml


class MockContext:

    def __init__(self):
        self.launch_configurations = {}

    def perform_substitution(self, sub):
        return sub.perform(None)


def test_set_param_is_scoped():
    lc = LaunchContext()
    push_conf = PushLaunchConfigurations()
    pop_conf = PopLaunchConfigurations()
    set_param = SetParametersFromFile('./example_parameters_0.yaml')

    push_conf.execute(lc)
    set_param.execute(lc)
    assert lc.launch_configurations == {'global_params': ['./example_parameters_0.yaml']}
    pop_conf.execute(lc)
    assert lc.launch_configurations == {}


def test_set_param_with_node():
    lc = MockContext()
    node_1 = Node(
        package='asd',
        executable='bsd',
        name='my_node',
        namespace='my_ns',
        parameters=[{'asd': 'bsd1'}]
    )

    node_2 = Node(
        package='asd',
        executable='bsd',
        name='my_other_node',
        namespace='my_ns',
        parameters=[{'asd': 'bsd2'}]
    )
    param_file_path = os.path.dirname(os.path.abspath(__file__)) + '/example_parameters_1.yaml'
    set_param_file = SetParametersFromFile(param_file_path)
    set_param_single = SetParameter(name='my_param', value='my_value')
    set_param_file.execute(lc)
    set_param_single.execute(lc)
    node_1._perform_substitutions(lc)
    node_2._perform_substitutions(lc)

    actual_command_1 = [perform_substitutions(lc, item) for item in
                        node_1.cmd if isinstance(item[0], TextSubstitution)]

    actual_command_2 = [perform_substitutions(lc, item) for item in
                        node_2.cmd if isinstance(item[0], TextSubstitution)]

    assert actual_command_1[3] == '--params-file'
    assert os.path.isfile(actual_command_1[4])
    assert actual_command_1[5] == '-p'
    assert actual_command_1[6] == 'my_param:=my_value'
    assert actual_command_1[7] == '--params-file'
    assert os.path.isfile(actual_command_1[8])

    expected_param_file = {
        '/**': {
            'ros__parameters': {'param_1_name': 10}
        },
        '/my_ns': {
            'my_node': {
                'ros__parameters': {'param_2_name': 20}
            },
            'my_other_node': {
                'ros__parameters': {'param_3_name': 30}
            }
        }
    }

    with open(actual_command_1[4], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == expected_param_file

    with open(actual_command_1[8], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == {
            '/my_ns/my_node': {
                'ros__parameters': {'asd': 'bsd1'}
            }
        }

    assert actual_command_2[3] == '--params-file'
    assert os.path.isfile(actual_command_2[4])
    assert actual_command_2[5] == '-p'
    assert actual_command_2[6] == 'my_param:=my_value'
    assert actual_command_2[7] == '--params-file'
    assert os.path.isfile(actual_command_2[8])

    with open(actual_command_2[4], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == expected_param_file

    with open(actual_command_2[8], 'r') as h:
        expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
        assert expanded_parameters_dict == {
            '/my_ns/my_other_node': {
                'ros__parameters': {'asd': 'bsd2'}
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
    param_file_path = os.path.dirname(os.path.abspath(__file__)) + '/example_parameters_1.yaml'
    set_param_1 = SetParametersFromFile(param_file_path)
    set_param_1.execute(lc)
    request = get_composable_node_load_request(node_description, lc)
    parameters = request.parameters
    assert len(parameters) == 3
    assert parameters[0].name == 'param_1_name'
    assert parameters[0].value.integer_value == 10
    assert parameters[1].name == 'param_2_name'
    assert parameters[1].value.integer_value == 20
    assert parameters[2].name == 'asd'
    assert parameters[2].value.string_value == 'bsd'

    lc = MockContext()
    node_description = ComposableNode(
        package='asd',
        plugin='my_plugin',
        name='my_node_2',
        namespace='my_ns',
        parameters=[{'asd': 'bsd'}]
    )
    param_file_path = os.path.dirname(os.path.abspath(__file__)) + '/example_parameters_1.yaml'
    set_param_1 = SetParametersFromFile(param_file_path)
    set_param_1.execute(lc)
    request = get_composable_node_load_request(node_description, lc)
    parameters = request.parameters
    assert len(parameters) == 2
    assert parameters[0].name == 'param_1_name'
    assert parameters[0].value.integer_value == 10
    assert parameters[1].name == 'asd'
    assert parameters[1].value.string_value == 'bsd'


def test_set_param_with_allow_substs_false():
    """Test that substitutions are not processed when allow_substs=False."""
    lc = LaunchContext()
    # Create a temporary parameter file with substitutions
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(
            """/**:
    ros__parameters:
        param_subst: $(env VAR_VALUE default_value)
        param_normal: 42
"""
        )
        temp_file = f.name

    try:
        set_param = SetParametersFromFile(temp_file, allow_substs=False)
        set_param.execute(lc)

        # The file path should be stored as-is (substitutions not processed)
        stored_path = lc.launch_configurations["global_params"][0]
        assert stored_path == temp_file

        # Verify the original file still contains the substitution syntax
        with open(temp_file, "r") as f:
            content = f.read()
            assert "$(env VAR_VALUE default_value)" in content
    finally:
        os.unlink(temp_file)


def test_set_param_with_allow_substs_true():
    """Test that substitutions are processed when allow_substs=True."""
    lc = LaunchContext()
    # Set an environment variable for substitution
    os.environ["VAR_VALUE"] = "substituted_value"

    # Create a temporary parameter file with substitutions
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(
            """/**:
    ros__parameters:
        param_subst: $(env VAR_VALUE default_value)
        param_normal: 42
"""
        )
        temp_file = f.name

    try:
        set_param = SetParametersFromFile(temp_file, allow_substs=True)
        set_param.execute(lc)

        # The stored path should be a temporary file (different from original)
        stored_path = lc.launch_configurations["global_params"][0]
        assert stored_path != temp_file
        assert os.path.isfile(stored_path)

        # Verify the substituted file has the substitution resolved
        with open(stored_path, "r") as f:
            content = f.read()
            assert "substituted_value" in content
            assert "$(env VAR_VALUE default_value)" not in content

        # Verify it's valid YAML
        with open(stored_path, "r") as f:
            params = yaml.safe_load(f)
            assert (
                params["/**"]["ros__parameters"]["param_subst"] == "substituted_value"
            )
            assert params["/**"]["ros__parameters"]["param_normal"] == 42
    finally:
        if "VAR_VALUE" in os.environ:
            del os.environ["VAR_VALUE"]
        os.unlink(temp_file)
        # Clean up temporary file if it exists
        stored_path = lc.launch_configurations.get("global_params", [None])[0]
        if stored_path and stored_path != temp_file and os.path.isfile(stored_path):
            os.unlink(stored_path)


def test_set_param_with_allow_substs_invalid_yaml():
    """Test that invalid YAML after substitution raises SubstitutionFailure."""
    lc = LaunchContext()
    # Set an environment variable that will create invalid YAML when substituted
    os.environ["VAR_VALUE"] = "[unclosed"

    # Create a temporary parameter file with substitutions that will create invalid YAML
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(
            """/**:
    ros__parameters:
        param_subst: $(env VAR_VALUE)
        param_normal: 42
"""
        )
        temp_file = f.name

    try:
        set_param = SetParametersFromFile(temp_file, allow_substs=True)
        # This should raise SubstitutionFailure because the substituted YAML is invalid
        with pytest.raises(SubstitutionFailure):
            set_param.execute(lc)
    finally:
        if "VAR_VALUE" in os.environ:
            del os.environ["VAR_VALUE"]
        os.unlink(temp_file)


def test_set_param_with_allow_substs_default_value():
    """Test that substitutions use default value when env var is not set."""
    lc = LaunchContext()
    # Ensure the environment variable is not set
    if "VAR_VALUE" in os.environ:
        del os.environ["VAR_VALUE"]

    # Create a temporary parameter file with substitutions and default value
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(
            """/**:
    ros__parameters:
        param_subst: $(env VAR_VALUE default_value)
        param_normal: 42
"""
        )
        temp_file = f.name

    try:
        set_param = SetParametersFromFile(temp_file, allow_substs=True)
        set_param.execute(lc)

        # Verify the substituted file uses the default value
        stored_path = lc.launch_configurations["global_params"][0]
        with open(stored_path, "r") as f:
            params = yaml.safe_load(f)
            assert params["/**"]["ros__parameters"]["param_subst"] == "default_value"
    finally:
        os.unlink(temp_file)
        # Clean up temporary file if it exists
        stored_path = lc.launch_configurations.get("global_params", [None])[0]
        if stored_path and stored_path != temp_file and os.path.isfile(stored_path):
            os.unlink(stored_path)
