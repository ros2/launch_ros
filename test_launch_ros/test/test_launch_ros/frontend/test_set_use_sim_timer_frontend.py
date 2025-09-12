# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from launch import LaunchService
from launch.frontend import Parser
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import SetUseSimTime
import pytest


yaml_file = textwrap.dedent(
    r"""
    launch:
        - let:
            name: sim_time
            value: '1'
        - set_use_sim_time:
            value: $(var sim_time)
    """
)


xml_file = textwrap.dedent(
    r"""
    <launch>
        <let name="sim_time" value="1" />
        <set_use_sim_time value="$(var sim_time)" />
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
def test_set_use_sim_timer(file):
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    lc = ls.context
    assert len(ld.entities) == 2
    assert isinstance(ld.entities[1], SetUseSimTime)
    assert perform_typed_substitution(lc, ld.entities[1].value, bool) is True
