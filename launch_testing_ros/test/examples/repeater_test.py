# Copyright 2023 Open Source Robotics Foundation, Inc.
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
import launch_ros
import launch_testing.actions
from launch_testing_ros.wait_for_topics import WaitForTopics
from launch_testing_ros import repeater

import pytest

import rclpy

from std_msgs.msg import String


@pytest.mark.rostest
def generate_test_description():

    # path_to_test = os.path.dirname(__file__)
    repeater_node = launch_ros.actions.Node(
        package="launch_testing_ros",
        executable="repeater",
        # arguments=[os.path.join(path_to_test, 'repeater.py')],
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    return (
        launch.LaunchDescription([
            repeater_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'repeater': repeater_node
        }
    )


class Testrepeater(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_repeater')
        self.publisher = self.node.create_publisher(String, "input", qos_profile=10)
        self.topic_list = [('output', String)]
        self.wait_for_topics = WaitForTopics(self.topic_list, timeout=10.0, start_subscribers=True)

    def tearDown(self):
        self.node.destroy_node()

    def test_repeater_translates_hello(self, launch_service, repeater, proc_output):
        # Expect the repeater to reply with "Hello World"
        msg = String()
        msg.data = "Hello"
        self.assertTrue(self.wait_for_topics.wait(start_subscribers=False))
        self.publisher.publish(msg)

        received_output = self.wait_for_topics.messages_received()['output']
        self.assertEqual(len(received_output), 1)
        self.assertEqual(received_output[0].data, "Hello World")

    def test_repeater_translates_knock(self, launch_service, repeater, proc_output):
        # Expect the repeater to reply with "Who's There?"
        msg = String()
        msg.data = "Knock Knock"

        self.assertTrue(self.wait_for_topics.wait(start_subscribers=False))
        self.publisher.publish(msg)

        received_output = self.wait_for_topics.messages_received()['output']
        self.assertEqual(len(received_output), 1)
        self.assertEqual(received_output[0].data, "Who's there")

    def test_repeater_ignores_foobar(self, launch_service, repeater, proc_output):
        # Expect the repeater to not reply"
        msg = String()
        msg.data = "FooBar"

        self.assertFalse(self.wait_for_topics.wait(start_subscribers=False))
        self.publisher.publish(msg)

        self.assertFalse('output' in self.wait_for_topics.messages_received())
        self.assertEqual(self.wait_for_topics.topics_not_received(), self.topic_list)
