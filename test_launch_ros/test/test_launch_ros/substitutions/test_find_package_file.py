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

"""Test for the FindPackageFile substitution."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError

from launch import LaunchContext
from launch.substitutions import SubstitutionFailure
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageFile

import pytest


def test_find_package_file():
    sub = FindPackageFile('launch_ros', 'package.xml')
    context = LaunchContext()
    result_path = Path(sub.perform(context))
    assert result_path.is_file()
    assert result_path.parent.name == 'launch_ros'
    assert result_path.name == 'package.xml'


def test_find_package_file_nested_path():
    sub = FindPackageFile('ament_index_python', 'environment/ament_index-argcomplete.bash')
    context = LaunchContext()
    result_path = Path(sub.perform(context))
    assert result_path.is_file()


def test_find_package_file_with_substitutions():
    sub = FindPackageFile(
        TextSubstitution(text='launch_ros'),
        [TextSubstitution(text='pack'), TextSubstitution(text='age.xml')],
    )
    context = LaunchContext()
    result_path = Path(sub.perform(context))
    assert result_path.is_file()
    assert result_path.name == 'package.xml'


def test_find_package_file_missing_file():
    sub = FindPackageFile('launch_ros', 'file_that_certainly_does_not_exist.txt')
    context = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        sub.perform(context)


def test_find_package_file_absolute_path():
    sub = FindPackageFile('launch_ros', '/package.xml')
    context = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        sub.perform(context)


def test_find_package_file_missing_package():
    sub = FindPackageFile('package_that_certainly_does_not_exist', 'package.xml')
    context = LaunchContext()
    with pytest.raises(PackageNotFoundError):
        sub.perform(context)


def test_find_package_file_parse():
    cls, kwargs = FindPackageFile.parse(['launch_ros', 'package.xml'])
    assert cls is FindPackageFile
    assert set(kwargs.keys()) == {'package', 'file'}


def test_find_package_file_parse_wrong_arg_count():
    with pytest.raises(AttributeError):
        FindPackageFile.parse(['launch_ros'])
