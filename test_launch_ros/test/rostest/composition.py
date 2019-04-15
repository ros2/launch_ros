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

import os
import unittest

import ament_index_python
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
import launch_testing
import launch_testing.asserts


def generate_test_description(ready_fn):
    launch_description = LaunchDescription()

    launch_description.add_action(
        ExecuteProcess(
            cmd=[
                os.path.join(
                    ament_index_python.get_package_prefix('test_launch_ros'),
                    'bin',
                    'mock-composable-container'),
            ],
        )
    )

    launch_description.add_action(
        OpaqueFunction(function=lambda context: ready_fn())
    )
    return launch_description, locals()


class TestComposition(unittest.TestCase):

    def test_hello_world(self):
        raise Exception(repr(self.proc_info._proc_info_handler._proc_info))
        proc = self.proc_info.processes()[0]
        launch_testing.asserts.assertInStdout(self.proc_output, 'Hello world', proc)


# @launch_testing.post_shutdown_test()
# class TestTwoExecutablesAfterShutdown(unittest.TestCase):
# 
#     def @TEST_NAME@(self, executable_under_test):
#         """Test that the executable under test finished cleanly."""
# launch_testing.asserts.assertExitCodes(self.proc_info, process=executable_under_test)
