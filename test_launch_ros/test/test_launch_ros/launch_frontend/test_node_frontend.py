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

from launch_frontend import Parser

import pytest

xml_file = \
    """\
    <launch>
        <node package="demo_nodes_py" executable="talker_qos" output="screen" name="my_node" namespace="my_ns" args="--number_of_cycles 1">
            <param name="param1" value="ads"/>
            <param name="param_group1">
                <param name="param_group2">
                    <param name="param2" value="2"/>
                </param>
                <param name="param3" value="2, 5, 8" value-sep=", "/>
            </param>
            <env name="var" value="1"/>
        </node>
    </launch>
"""  # noqa: E501
xml_file = textwrap.dedent(xml_file)
yaml_file = \
    """\
    launch:
        - node:
            package: demo_nodes_py
            executable: talker_qos
            output: screen
            name: my_node
            namespace: my_ns
            args: '--number_of_cycles 1'
            param:
                -   name: param1
                    value: ads
                -   name: param_group1
                    param:
                    -   name: param_group2
                        param:
                        -   name: param2
                            value: 2
                    -   name: param3
                        value: [2, 5, 8]
            env:
                -   name: var
                    value: '1'
    """
yaml_file = textwrap.dedent(yaml_file)


@pytest.mark.parametrize('file', (xml_file, yaml_file))
def test_node_frontend(file):
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert(0 == ls.run())
