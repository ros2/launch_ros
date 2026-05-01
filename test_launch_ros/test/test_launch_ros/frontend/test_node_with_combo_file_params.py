# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2020 Open Avatar Inc.
# Copyright 2026 Metro Robots
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
import pathlib
import textwrap

from launch import LaunchService
from launch.frontend import Parser

from launch_ros.utilities import evaluate_parameters

import yaml

parent_folder = pathlib.Path(__file__).parent
# Escape backslashes if any to keep them after parsing takes place
yaml_params1 = str(parent_folder / 'params.yaml').replace('\\', '\\\\')
yaml_params2 = str(parent_folder / 'params2.yaml').replace('\\', '\\\\')


def test_launch_file_params_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <node pkg="demo_nodes_py" exec="talker_qos" name="my_talker" args="--number_of_cycles 1">
                <param>
                  <file path="{}"/>
                  <file path="{}"/>
                </param>
            </node>
        </launch>
        """.format(yaml_params1, yaml_params2))  # noqa: E501

    with io.StringIO(xml_file) as f:
        check_launch_node(f)


def test_launch_file_params_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - node:
                pkg: demo_nodes_py
                exec: talker_qos
                name: my_talker
                args: '--number_of_cycles 1'
                param:
                - file:
                    - path: {}
                    - path: {}
        """.format(yaml_params1, yaml_params2))  # noqa: E501

    with io.StringIO(yaml_file) as f:
        check_launch_node(f)


def check_launch_node(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    evaluated_parameters = evaluate_parameters(
        ls.context,
        ld.describe_sub_entities()[0]._Node__parameters
    )
    assert len(evaluated_parameters) == 1
    assert isinstance(evaluated_parameters[0], pathlib.Path)

    param_path = evaluated_parameters[0]
    file_params = yaml.safe_load(open(param_path))
    ros_params = file_params.get('my_ns', {}).get('my_node', {}).get('ros__parameters', {})
    assert ros_params['param_from_file_1'] == 5      # Loaded from params1 overloaded by params2
    assert ros_params['param_from_file_2'] == 'asd'  # Loaded from params1
    assert ros_params['param_from_file_3'] == 3.14   # Loaded from params2
