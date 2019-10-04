# Copyright 2019 Apex.AI, Inc.
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
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing_ros

import pytest

import std_msgs.msg


@pytest.mark.rostest
def generate_test_description(ready_fn):
    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through
    path_to_test = os.path.dirname(__file__)

    talker_node = launch_ros.actions.Node(
        node_executable=sys.executable,
        arguments=[os.path.join(path_to_test, 'talker.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        remappings=[('chatter', 'talker_chatter')]
    )

    listener_node = launch_ros.actions.Node(
        node_executable=sys.executable,
        arguments=[os.path.join(path_to_test, 'listener.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        remappings=[('chatter', 'listener_chatter')]
    )

    return (
        launch.LaunchDescription([
            talker_node,
            listener_node,
            # Start tests right away - no need to wait for anything
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]),
        {
            'talker': talker_node,
            'listener': listener_node,
        }
    )


class TestTalkerListenerLink(unittest.TestCase):

    def test_talker_transmits(self, launch_service, talker):
        # Get launch context ROS node
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node

        # Expect the talker to publish strings on '/talker_chatter' and also write to stdout
        msgs_rx = []

        sub = node.create_subscription(
            std_msgs.msg.String,
            'talker_chatter',
            lambda msg: msgs_rx.append(msg),
            10
        )
        try:
            # Wait until the talker transmits two messages over the ROS topic
            end_time = time.time() + 10
            while time.time() < end_time:
                time.sleep(1.0)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2)

            # Make sure the talker also output the same data via stdout
            for msg in msgs_rx:
                self.proc_output.assertWaitFor(
                    expected_output=msg.data, process=talker
                )
        finally:
            node.destroy_subscription(sub)

    def test_listener_receives(self, launch_service, listener):
        # Get launch context ROS node
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node

        pub = node.create_publisher(
            std_msgs.msg.String,
            'listener_chatter',
            10
        )
        try:
            # Publish a unique message on /chatter and verify that the listener
            # gets it and prints it
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())
            for _ in range(10):
                pub.publish(msg)
                success = self.proc_output.waitFor(
                    expected_output=msg.data,
                    process=listener,
                    timeout=1.0,
                )
                if success:
                    break
            assert success, 'Waiting for output timed out'
        finally:
            node.destroy_publisher(pub)

    def test_fuzzy_data(self, launch_service, listener):
        # Get launch context ROS node
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node

        # This test shows how to insert a node in between the talker and the listener to
        # change the data.  Here we're going to change 'Hello World' to 'Aloha World'
        def data_mangler(msg):
            msg.data = msg.data.replace('Hello', 'Aloha')
            return msg

        republisher = launch_testing_ros.DataRepublisher(
            node,
            'talker_chatter',
            'listener_chatter',
            std_msgs.msg.String,
            data_mangler
        )
        try:
            # Spin for a few seconds until we've republished some mangled messages
            end_time = time.time() + 10
            while time.time() < end_time:
                time.sleep(1.0)
                if republisher.get_num_republished() > 2:
                    break

            self.assertGreater(republisher.get_num_republished(), 2)

            # Sanity check that we're changing 'Hello World'
            self.proc_output.assertWaitFor('Aloha World')

            # Check for the actual messages we sent
            for msg in republisher.get_republished():
                self.proc_output.assertWaitFor(msg.data, listener)
        finally:
            republisher.shutdown()
