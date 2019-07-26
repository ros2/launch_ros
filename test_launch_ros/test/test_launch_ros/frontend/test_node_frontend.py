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
import pathlib
import textwrap

from launch import LaunchService
from launch.frontend import Parser

from launch_ros.utilities import evaluate_parameters

import pytest

yaml_params = str(pathlib.Path(__file__).parent / 'params.yaml')
xml_file = \
    r"""
    <launch>
        <let name="a_string" value="\'[2, 5, 8]\'"/>
        <let name="a_list" value="[2, 5, 8]"/>
        <node pkg="demo_nodes_py" exec="talker_qos" output="screen" name="my_node" namespace="my_ns" args="--number_of_cycles 1">
            <param name="param1" value="ads"/>
            <param name="param_group1">
                <param name="param_group2">
                    <param name="param2" value="2"/>
                </param>
                <param name="param3" value="2, 5, 8" value-sep=", "/>
                <param name="param4" value="$(var a_list)"/>
                <param name="param5" value="$(var a_string)"/>
                <param name="param6" value="2., 5., 8." value-sep=", "/>
                <param name="param7" value="'2', '5', '8'" value-sep=", "/>
                <param name="param8" value="''2'', ''5'', ''8''" value-sep=", "/>
                <param name="param9" value="\'2\', \'5\', \'8\'" value-sep=", "/>
                <param name="param10" value="''asd'', ''bsd'', ''csd''" value-sep=", "/>
                <param name="param11" value="'\asd', '\bsd', '\csd'" value-sep=", "/>
            </param>
            <param from="{}"/>
            <env name="var" value="1"/>
        </node>
    </launch>
    """.format(yaml_params)  # noqa: E501
xml_file = textwrap.dedent(xml_file)
yaml_file = \
    r"""
    launch:
        - let:
            name: 'a_string'
            value: "'[2, 5, 8]'"
        - let:
            name: 'a_list'
            value: '[2, 5, 8]'
        - node:
            pkg: demo_nodes_py
            exec: talker_qos
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
                    -   name: param6
                        value: [2., 5., 8.]
                    -   name: param7
                        value: ['2', '5', '8']
                    -   name: param8
                        value: ["'2'", "'5'", "'8'"]
                    -   name: param9
                        value: ["\\'2\\'", "\\'5\\'", "\\'8\\'"]
                    -   name: param10
                        value: ["'asd'", "'bsd'", "'csd'"]
                    -   name: param11
                        value: ['\asd', '\bsd', '\csd']
                -   from: {}
            env:
                -   name: var
                    value: '1'
    """.format(yaml_params)  # noqa: E501
yaml_file = textwrap.dedent(yaml_file)


@pytest.mark.parametrize('file', (xml_file, yaml_file))
def test_node_frontend(file):
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert(0 == ls.run())
    evaluated_parameters = evaluate_parameters(
        ls.context,
        ld.describe_sub_entities()[2]._Node__parameters
    )
    assert isinstance(evaluated_parameters[0], pathlib.Path)
    assert isinstance(evaluated_parameters[1], dict)
    param_dict = evaluated_parameters[1]
    assert 'param1' in param_dict
    assert param_dict['param1'] == 'ads'
    assert 'param_group1.param_group2.param2' in param_dict
    assert 'param_group1.param3' in param_dict
    assert 'param_group1.param4' in param_dict
    assert 'param_group1.param5' in param_dict
    assert 'param_group1.param6' in param_dict
    assert 'param_group1.param7' in param_dict
    assert 'param_group1.param8' in param_dict
    assert 'param_group1.param9' in param_dict
    assert 'param_group1.param10' in param_dict
    assert 'param_group1.param11' in param_dict
    assert param_dict['param_group1.param_group2.param2'] == 2
    assert param_dict['param_group1.param3'] == (2, 5, 8)
    assert param_dict['param_group1.param4'] == (2, 5, 8)
    assert param_dict['param_group1.param5'] == '[2, 5, 8]'
    assert param_dict['param_group1.param6'] == (2., 5., 8.)
    assert param_dict['param_group1.param7'] == ('2', '5', '8')
    assert param_dict['param_group1.param8'] == ("'2'", "'5'", "'8'")
    assert param_dict['param_group1.param9'] == ("'2'", "'5'", "'8'")
    assert param_dict['param_group1.param10'] == ("'asd'", "'bsd'", "'csd'")
    assert param_dict['param_group1.param11'] == ('asd', 'bsd', 'csd')
