# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Example of how to parse an xml."""

import io
import textwrap

from launch import LaunchService
from launch.frontend import Parser

import pytest

xml_file = \
    r"""
    <launch>
        <push-ros-namespace namespace="asd"/>
    </launch>
    """
xml_file = textwrap.dedent(xml_file)
yaml_file = \
    r"""
    launch:
        - push-ros-namespace:
            namespace: 'asd'
    """
yaml_file = textwrap.dedent(yaml_file)


@pytest.mark.parametrize('file', (xml_file, yaml_file))
def test_node_frontend(file):
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert 'ros_namespace' in ls.context.launch_configurations
    assert '/asd' == ls.context.launch_configurations['ros_namespace']
