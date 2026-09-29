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
from pathlib import Path
import textwrap

from launch import LaunchService
from launch.frontend import Parser
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions

import pytest


def test_ament_index_resource_substitution_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: launch_ros_prefix
                value: $(ament-index-resource packages launch_ros)
        """
    )
    with io.StringIO(yaml_file) as f:
        check_ament_index_resource_substitution(f)


def test_ament_index_resource_substitution_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="launch_ros_prefix" value="$(ament-index-resource packages launch_ros)"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_ament_index_resource_substitution(f)


def check_ament_index_resource_substitution(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    let, = ld.describe_sub_entities()
    assert perform(let.name) == 'launch_ros_prefix'
    assert Path(perform(let.value)).is_dir()


def test_ament_index_resource_substitution_yaml_nonexistent_name():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: bad_name
                value: $(ament-index-resource packages package_that_certainly_does_not_exist)
        """
    )
    with io.StringIO(yaml_file) as f:
        check_ament_index_resource_substitution_failure(f)


def test_ament_index_resource_substitution_yaml_nonexistent_type():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: bad_type
                value: $(ament-index-resource resource_type_that_does_not_exist launch_ros)
        """
    )
    with io.StringIO(yaml_file) as f:
        check_ament_index_resource_substitution_failure(f)


def test_ament_index_resource_substitution_xml_nonexistent_name():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="launch_ros_prefix"
                 value="$(ament-index-resource packages package_that_certainly_does_not_exist)"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_ament_index_resource_substitution_failure(f)


def test_ament_index_resource_substitution_xml_nonexistent_type():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="launch_ros_prefix"
                 value="$(ament-index-resource resource_type_that_does_not_exist launch_ros)"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_ament_index_resource_substitution_failure(f)


def check_ament_index_resource_substitution_failure(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 != ls.run()

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    let, = ld.describe_sub_entities()
    with pytest.raises(SubstitutionFailure):
        perform(let.value)
