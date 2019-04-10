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
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.util
import launch_testing_ros
import rclpy
import rclpy.context
import rclpy.executors
import std_msgs.msg


def generate_test_description(ready_fn):
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through
    talker_node = launch_ros.actions.Node(
        package='demo_nodes_py',
        node_executable='talker',
        env=proc_env,
        remappings=[('chatter', 'talker_chatter')],
    )

    listener_node = launch_ros.actions.Node(
        package='demo_nodes_py',
        node_executable='listener',
        env=proc_env,
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

    @classmethod
    def setUpClass(cls, proc_output, listener):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_node', context=cls.context)

        # The demo node listener has no synchronization to indicate when it's ready to start
        # receiving messages on the /chatter topic.  This plumb_listener method will attempt
        # to publish for a few seconds until it sees output
        publisher = cls.node.create_publisher(
            std_msgs.msg.String,
            'chatter'
        )
        msg = std_msgs.msg.String()
        msg.data = 'test message {}'.format(uuid.uuid4())
        for _ in range(5):
            try:
                publisher.publish(msg)
                proc_output.assertWaitFor(
                    expected_output=msg.data,
                    process=listener,
                    timeout=1.0
                )
            except AssertionError:
                continue
            except launch_testing.util.NoMatchingProcessException:
                continue
            else:
                return
        else:
            assert False, 'Failed to plumb chatter topic to listener process'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def spin_rclpy(self, timeout_sec):
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        try:
            executor.spin_once(timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self.node)

    def test_talker_transmits(self, talker):
        # Expect the talker to publish strings on '/talker_chatter' and also write to stdout
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.String,
            'talker_chatter',
            callback=lambda msg: msgs_rx.append(msg)
        )
        self.addCleanup(self.node.destroy_subscription, sub)

        # Wait until the talker transmits two messages over the ROS topic
        end_time = time.time() + 10
        while time.time() < end_time:
            self.spin_rclpy(1.0)
            if len(msgs_rx) > 2:
                break

        self.assertGreater(len(msgs_rx), 2)

        # Make sure the talker also output the same data via stdout
        for txt in [msg.data for msg in msgs_rx]:
            self.proc_output.assertWaitFor(
                expected_output=txt,
                process=talker
            )

    def test_listener_receives(self, listener):
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            'chatter'
        )
        self.addCleanup(self.node.destroy_publisher, pub)

        # Publish some unique messages on /chatter and verify that the listener gets them
        # and prints them
        for _ in range(5):
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())

            pub.publish(msg)
            self.proc_output.assertWaitFor(
                expected_output=msg.data,
                process=listener
            )

    def test_fuzzy_data(self, listener):
        # This test shows how to insert a node in between the talker and the listener to
        # change the data.  Here we're going to change 'Hello World' to 'Aloha World'
        def data_mangler(msg):
            msg.data = msg.data.replace('Hello', 'Aloha')
            return msg

        republisher = launch_testing_ros.DataRepublisher(
            self.node,
            'talker_chatter',
            'chatter',
            std_msgs.msg.String,
            data_mangler
        )
        self.addCleanup(republisher.shutdown)

        # Spin for a few seconds until we've republished some mangled messages
        end_time = time.time() + 10
        while time.time() < end_time:
            self.spin_rclpy(1.0)
            if republisher.get_num_republished() > 2:
                break

        self.assertGreater(republisher.get_num_republished(), 2)

        # Sanity check that we're changing 'Hello World'
        self.proc_output.assertWaitFor('Aloha World')

        # Check for the actual messages we sent
        for msg in republisher.get_republished():
            self.proc_output.assertWaitFor(msg.data, listener)
