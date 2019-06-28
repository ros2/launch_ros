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

from launch_ros.utilities import evaluate_parameters

import pytest

# Scaping the quote is needed in 'a_string' launch configuration, becuase of how the parser works.
# TODO(ivanpauno): Check if it's possible to avoid that.
xml_file = \
    """\
    <launch>
        <let name="a_string" value="\\'[2, 5, 8]\\'"/>
        <let name="a_list" value="[2, 5, 8]"/>
        <node package="demo_nodes_py" executable="talker_qos" output="screen" name="my_node" namespace="my_ns" args="--number_of_cycles 1">
            <param name="param1" value="ads"/>
            <param name="param_group1">
                <param name="param_group2">
                    <param name="param2" value="2"/>
                </param>
                <param name="param3" value="2, 5, 8" value-sep=", "/>
                <param name="param4" value="$(var a_list)"/>
                <param name="param5" value="$(var a_string)"/>
            </param>
            <env name="var" value="1"/>
        </node>
    </launch>
    """  # noqa: E501
xml_file = textwrap.dedent(xml_file)
yaml_file = \
    """\
    launch:
        - let:
            name: 'a_string'
            value: '\\"[2, 5, 8]\\"'
        - let:
            name: 'a_list'
            value: '[2, 5, 8]'
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
                    -   name: param4
                        value: $(var a_list)
                    -   name: param5
                        value: $(var a_string)
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
    lc = ls.context
    evaluated_parameters = evaluate_parameters(
        lc,
        ld.describe_sub_entities()[2]._Node__parameters
    )[0]
    assert isinstance(evaluated_parameters, dict)
    assert 'param1' in evaluated_parameters
    assert evaluated_parameters['param1'] == 'ads'
    assert 'param_group1.param_group2.param2' in evaluated_parameters
    assert 'param_group1.param3' in evaluated_parameters
    assert 'param_group1.param4' in evaluated_parameters
    assert 'param_group1.param5' in evaluated_parameters
    assert evaluated_parameters['param_group1.param_group2.param2'] == 2
    assert evaluated_parameters['param_group1.param3'] == (2, 5, 8)
    assert evaluated_parameters['param_group1.param4'] == (2, 5, 8)
    assert evaluated_parameters['param_group1.param5'] == '[2, 5, 8]'
