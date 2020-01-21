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
import sys
import textwrap

from launch import LaunchService
from launch.frontend import Parser

from launch_ros.utilities import evaluate_parameters

import pytest

yaml_params = str(pathlib.Path(__file__).parent / 'params.yaml')

# Escape backslashes if any to keep them after parsing takes place
yaml_params = yaml_params.replace('\\', '\\\\')
python_executable = sys.executable.replace('\\', '\\\\')

xml_file = \
    r"""
    <launch>
        <let name="a_string" value="\'[2, 5, 8]\'"/>
        <let name="a_list" value="[2, 5, 8]"/>
        <node pkg="demo_nodes_py" exec="talker_qos" output="screen" node-name="my_talker" namespace="my_ns" args="--number_of_cycles 1">
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
                <param name="param12" value=""/>
            </param>
            <param from="{}"/>
            <env name="var" value="1"/>
            <remap from="foo" to="bar"/>
            <remap from="baz" to="foobar"/>
        </node>
        <node exec="{}" args="-c 'import sys; print(sys.argv[1:])'" node-name="my_listener" namespace="my_ns" output="screen"/>
    </launch>
    """.format(yaml_params, python_executable)  # noqa: E501
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
            node-name: my_talker
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
                    -   name: param12
                        value: ''
                -   from: {}
            env:
                -   name: var
                    value: '1'
            remap:
                -   from: "foo"
                    to: "bar"
                -   from: "baz"
                    to: "foobar"
        - node:
            exec: {}
            output: screen
            namespace: my_ns
            node-name: my_listener
            args: -c 'import sys; print(sys.argv[1:])'
    """.format(yaml_params, python_executable)  # noqa: E501
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
    assert isinstance(evaluated_parameters[0], dict)
    assert isinstance(evaluated_parameters[1], dict)
    assert isinstance(evaluated_parameters[2], pathlib.Path)

    assert 'param1' in evaluated_parameters[0]
    assert evaluated_parameters[0]['param1'] == 'ads'

    param_dict = evaluated_parameters[1]
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
    assert param_dict['param_group1.param12'] == ''

    # Check remappings exist
    remappings = ld.describe_sub_entities()[2]._Node__remappings
    assert remappings is not None
    assert len(remappings) == 2

    listener_node_action = ld.describe_sub_entities()[3]
    listener_node_cmd = listener_node_action.process_details['cmd']
    assert [
        sys.executable, '-c', 'import sys; print(sys.argv[1:])'
    ] == listener_node_cmd[:3]
