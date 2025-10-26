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

import asyncio
import io
import pathlib
import textwrap

from launch import LaunchService
from launch.frontend import Parser
from launch.utilities import type_utils
from launch_ros.actions import LifecycleNode
from launch_ros.utilities import evaluate_parameters
import osrf_pycommon.process_utils

yaml_params = str(pathlib.Path(__file__).parent / 'params.yaml')

# Escape backslashes if any to keep them after parsing takes place
yaml_params = yaml_params.replace('\\', '\\\\')


def test_launch_frontend_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" output="screen" name="my_lc_talker" namespace="my_lc_ns" exec_name="my_lc_talker_process" ros_args="--log-level info --log-file-name filename">
                <param name="param1" value="ads"/>
                <param name="param_group1">
                    <param name="param_group2">
                        <param name="param2" value="2"/>
                    </param>
                    <param name="param3" value="2, 5, 8" value-sep=", "/>
                </param>
                <param from="{}"/>
                <env name="var" value="1"/>
                <remap from="foo" to="bar"/>
                <remap from="baz" to="foobar"/>
            </lifecycle_node>
            <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" output="screen" name="my_lc_auto_talker" namespace="my_lc_auto_ns" exec_name="my_lc_talker_auto_process" autostart="True"/>
        </launch>
        """.format(yaml_params))  # noqa: E501

    with io.StringIO(xml_file) as f:
        check_launch_lifecycle_node(f)


def test_launch_frontend_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - lifecycle_node:
                pkg: lifecycle
                exec: lifecycle_talker
                output: screen
                name: my_lc_talker
                namespace: my_lc_ns
                exec_name: my_lc_talker_process
                ros_args: "--log-level info --log-file-name filename"
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
                    -   from: {}
                env:
                    -   name: var
                        value: '1'
                remap:
                    -   from: "foo"
                        to: "bar"
                    -   from: "baz"
                        to: "foobar"
            - lifecycle_node:
                pkg: lifecycle
                exec: lifecycle_talker
                output: screen
                name: my_lc_auto_talker
                namespace: my_lc_auto_ns
                exec_name: my_lc_talker_auto_process
                autostart: True
        """.format(yaml_params))

    with io.StringIO(yaml_file) as f:
        check_launch_lifecycle_node(f)


def check_launch_lifecycle_node(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)

    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())

    lc_talker_node, lc_auto_talker_node = ld.describe_sub_entities()

    assert isinstance(lc_talker_node, LifecycleNode)
    assert not type_utils.perform_typed_substitution(
        ls.context, lc_talker_node.node_autostart, bool)

    assert isinstance(lc_auto_talker_node, LifecycleNode)
    assert type_utils.perform_typed_substitution(
        ls.context, lc_auto_talker_node.node_autostart, bool)

    evaluated_parameters = evaluate_parameters(ls.context, lc_talker_node._Node__parameters)

    assert len(evaluated_parameters) == 3
    assert isinstance(evaluated_parameters[0], dict)
    assert isinstance(evaluated_parameters[1], dict)
    assert isinstance(evaluated_parameters[2], pathlib.Path)

    assert len(evaluated_parameters[0]) == 1
    assert 'param1' in evaluated_parameters[0]
    assert evaluated_parameters[0]['param1'] == 'ads'

    param_dict = evaluated_parameters[1]
    assert len(param_dict) == 2
    assert 'param_group1.param_group2.param2' in param_dict
    assert 'param_group1.param3' in param_dict
    assert param_dict['param_group1.param_group2.param2'] == 2
    assert param_dict['param_group1.param3'] == [2, 5, 8]

    assert evaluated_parameters[2] == pathlib.PurePath(yaml_params)

    assert len(lc_auto_talker_node._Node__parameters) == 0

    # Check remappings exist
    remappings = lc_talker_node._Node__remappings
    assert remappings is not None
    assert len(remappings) == 2

    assert len(lc_auto_talker_node._Node__remappings) == 0

    timeout_sec = 5
    loop.run_until_complete(asyncio.sleep(timeout_sec))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    assert 0 == launch_task.result()

    talker_node_cmd_string = ' '.join(lc_talker_node.process_details['cmd'])
    assert '--ros-args --log-level info --log-file-name filename' in talker_node_cmd_string

    assert lc_talker_node.node_name == '/my_lc_ns/my_lc_talker'
    assert lc_auto_talker_node.node_name == '/my_lc_auto_ns/my_lc_auto_talker'
