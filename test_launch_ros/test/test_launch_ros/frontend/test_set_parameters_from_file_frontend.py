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

import io
import textwrap

from launch import LaunchContext
from launch.frontend import Parser
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import SetParametersFromFile

import pytest


yaml_file = textwrap.dedent(
    r"""
    launch:
        - set_parameters_from_file:
            filename: 'params.yaml'
            override: true
        - set_parameters_from_file:
            filename: 'other.yaml'
    """
)


xml_file = textwrap.dedent(
    r"""
    <launch>
        <set_parameters_from_file filename="params.yaml" override="true" />
        <set_parameters_from_file filename="other.yaml" />
    </launch>
    """
)


@pytest.mark.parametrize(
    'file',
    [
        pytest.param(yaml_file, id='YAML'),
        pytest.param(xml_file, id='XML'),
    ],
)
def test_set_parameters_from_file_frontend(file):
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], SetParametersFromFile)
    assert isinstance(ld.entities[1], SetParametersFromFile)

    lc = LaunchContext()
    # The 'override' attribute is parsed when present ...
    assert perform_typed_substitution(lc, ld.entities[0].override, bool) is True
    # ... and defaults to False when it is not.
    assert perform_typed_substitution(lc, ld.entities[1].override, bool) is False
