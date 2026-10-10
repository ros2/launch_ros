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

"""Test for the FindPackageFile substitution frontend."""

import io
from pathlib import Path
import textwrap

from launch import LaunchService
from launch.frontend import Parser
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions

import pytest


def test_find_package_file_substitution_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: launch_ros_package_xml
                value: $(find-pkg-file launch_ros package.xml)
        """
    )
    with io.StringIO(yaml_file) as f:
        check_find_package_file_substitution(f)


def test_find_package_file_substitution_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="launch_ros_package_xml"
                 value="$(find-pkg-file launch_ros package.xml)"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_find_package_file_substitution(f)


def check_find_package_file_substitution(file):
    """Check that the find-pkg-file frontend resolves the package.xml of launch_ros."""
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    let, = ld.describe_sub_entities()
    assert perform(let.name) == 'launch_ros_package_xml'
    result_path = Path(perform(let.value))
    assert result_path.is_file()
    assert result_path.name == 'package.xml'


def test_find_package_file_substitution_yaml_missing_file():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: bad_file
                value: $(find-pkg-file launch_ros file_that_certainly_does_not_exist.txt)
        """
    )
    with io.StringIO(yaml_file) as f:
        check_find_package_file_substitution_failure(f)


def test_find_package_file_substitution_xml_missing_file():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="bad_file"
                 value="$(find-pkg-file launch_ros file_that_certainly_does_not_exist.txt)"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_find_package_file_substitution_failure(f)


def check_find_package_file_substitution_failure(file):
    """Check that an unresolvable find-pkg-file frontend fails the launch."""
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
