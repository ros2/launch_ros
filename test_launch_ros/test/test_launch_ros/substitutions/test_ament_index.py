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

"""Test for the AmentIndexResource substitution."""

from pathlib import Path

from launch import LaunchContext
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch_ros.substitutions import AmentIndexResource

import pytest


def test_ament_index_resource():
    sub = AmentIndexResource('packages', 'launch_ros')
    context = LaunchContext()
    resource_path = Path(sub.perform(context))
    assert resource_path.is_dir()


def test_ament_index_resource_nonexistent_name():
    sub = AmentIndexResource('packages', 'package_that_certainly_does_not_exist')
    context = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        sub.perform(context)


def test_ament_index_resource_nonexistent_type():
    sub = AmentIndexResource('resource_type_that_certainly_does_not_exist', 'launch_ros')
    context = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        sub.perform(context)
