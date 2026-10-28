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

"""Tests for ros2launch.api.dump_params_of_a_launch_file."""

import io
from unittest.mock import patch

from ros2launch.api import dump_params_of_a_launch_file

import yaml


def test_dump_params_of_a_launch_file_api(tmp_path):
    launch_path = tmp_path / 'dump_params_test.launch.py'
    launch_path.write_text(
        """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='demo',
            parameters=[{'foo': 1, 'bar': 'baz'}],
            ros_arguments=['-p', 'extra:=2'],
        ),
    ])
"""
    )

    buf = io.StringIO()
    with patch('sys.stdout', buf):
        rc = dump_params_of_a_launch_file(launch_file_path=str(launch_path))
    assert rc == 0
    dumped = buf.getvalue()
    assert not dumped.lstrip().startswith('[')
    data = yaml.safe_load(dumped)
    assert '/demo/talker' in data
    params = data['/demo/talker']['ros__parameters']
    assert params['foo'] == 1
    assert params['bar'] == 'baz'
    assert params['use_sim_time'] is True
    assert params['extra'] == 2
