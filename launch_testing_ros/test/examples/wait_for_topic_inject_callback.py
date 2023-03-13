# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import sys
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
from std_msgs.msg import String

from axel_launch_testing_ros import WaitForTopics


def generate_node():
    """Return node and remap the topic based on the index provided."""
    path_to_test = os.path.dirname(__file__)
    return launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(path_to_test, "repeater.py")],
        name="demo_node",
        additional_env={"PYTHONUNBUFFERED": "1"},
    )


def trigger_callback():
    os.system(f'ros2 topic pub --once /input std_msgs/msg/String "data: Hello World"')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    description = [generate_node(), launch_testing.actions.ReadyToTest()]
    return launch.LaunchDescription(description)


# TODO: Test cases fail on Windows debug builds
# https://github.com/ros2/launch_ros/issues/292
if os.name != "nt":

    class TestFixture(unittest.TestCase):
        def test_topics_successful(self):
            """All the supplied topics should be read successfully."""
            topic_list = [("output", String)]
            expected_topics = {"output"}

            # Method 1 : Using the magic methods and 'with' keyword
            with WaitForTopics(
                    topic_list, timeout=2000.0, callback=trigger_callback
            ) as wait_for_node_object_1:
                assert wait_for_node_object_1.topics_received() == expected_topics
                assert wait_for_node_object_1.topics_not_received() == set()