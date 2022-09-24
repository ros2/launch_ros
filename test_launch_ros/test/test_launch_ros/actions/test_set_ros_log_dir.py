# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetROSLogDir Action."""

import io
import os
import textwrap

from launch import LaunchContext
from launch.actions import PopEnvironment
from launch.actions import PushEnvironment
from launch.frontend import Parser

from launch_ros.actions import SetROSLogDir


def test_set_ros_log_dir_constructor():
    """Test the constructor for SetROSLogDir class."""
    SetROSLogDir(new_log_dir='')


def test_set_ros_log_dir_relative(tmpdir):
    """Test adding a relative path to the ROS log dir."""
    lc = LaunchContext()

    PushEnvironment().visit(lc)
    try:
        initial_ros_log_dir = os.path.join(tmpdir, 'test_ros_log_dir')
        os.environ['ROS_LOG_DIR'] = initial_ros_log_dir
        SetROSLogDir('relative').visit(lc)
        assert os.environ['ROS_LOG_DIR'] == os.path.join(initial_ros_log_dir, 'relative')
    finally:
        PopEnvironment().visit(lc)


def test_set_ros_log_dir_absolute(tmpdir):
    """Test setting an absolute path to the ROS log dir."""
    lc = LaunchContext()

    PushEnvironment().visit(lc)
    try:
        initial_ros_log_dir = os.path.join(tmpdir, 'test_ros_log_dir')
        os.environ['ROS_LOG_DIR'] = initial_ros_log_dir
        new_ros_log_dir = os.path.join(tmpdir, 'test_set_ros_log_dir_absolute')
        SetROSLogDir(new_ros_log_dir).visit(lc)
        assert os.environ['ROS_LOG_DIR'] == new_ros_log_dir
    finally:
        PopEnvironment().visit(lc)


def test_set_ros_log_dir_frontend():
    """Test the frontend parsing of SetROSLogDir."""
    xml_file = textwrap.dedent(r"""
        <launch>
            <set_ros_log_dir new_log_dir="relative" />
        </launch>
    """)

    with io.StringIO(xml_file) as f:
        root_entity, parser = Parser.load(f)
        ld = parser.parse_description(root_entity)
        assert len(ld.entities) == 1
        assert isinstance(ld.entities[0], SetROSLogDir)
