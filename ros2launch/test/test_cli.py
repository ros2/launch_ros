# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import contextlib
import os
import re
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rmw_implementation import get_available_rmw_implementations


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    additional_env = {
        'RMW_IMPLEMENTATION': rmw_implementation, 'PYTHONUNBUFFERED': '1'
    }

    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2LaunchCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
    ):
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_patterns=[r'.*(\[launch\]|Hello World).*'],
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def ros2_launch_command(self, arguments):
            launch_command_action = ExecuteProcess(
                cmd=['ros2', 'launch', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2launch-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, launch_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as launch_command:
                yield launch_command
        cls.launch_command = ros2_launch_command

        path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
        cls.main_launch_file = os.path.join(path_to_fixtures, 'main.launch.py')

    @launch_testing.markers.retry_on_failure(times=5)
    def test_launch_nonunique_node_names(self):
        with self.launch_command(arguments=[self.main_launch_file]) as launch_command:
            assert launch_command.wait_for_shutdown(timeout=10)
        assert launch_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '[WARNING] [python3-3]: there are now 2 nodes with the name /talker',
                '[WARNING] [python3-4]: there are now 2 nodes with the name /listener',
                re.compile(r'\[INFO\] \[python3-1\]: process started with pid \[\d+\]'),
                re.compile(r'\[INFO\] \[python3-2\]: process started with pid \[\d+\]'),
                re.compile(r'\[INFO\] \[python3-3\]: process started with pid \[\d+\]'),
                re.compile(r'\[INFO\] \[python3-4\]: process started with pid \[\d+\]')
            ],
            text=launch_command.output
        )
