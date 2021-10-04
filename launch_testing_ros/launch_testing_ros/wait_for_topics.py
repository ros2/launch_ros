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

from threading import Event
from threading import Lock
from threading import Thread

import rclpy
from rclpy.node import Node


class WaitForTopics:
    """Wait to receive messages on supplied nodes."""

    def __init__(self, topic_tuples):
        self.__ros_context = None
        self.__ros_node = None
        self.topic_tuples = topic_tuples
        self.start()

    def start(self):
        self.__ros_context = rclpy.Context()
        rclpy.init()
        self.__ros_node = MakeTestNode(name='test_node')

    def wait(self, timeout=5.0):
        self.__ros_node.start_subscribers(self.topic_tuples)
        return self.__ros_node.msg_event_object.wait(timeout)

    def shutdown(self):
        rclpy.shutdown()


class MakeTestNode(Node):
    """Mock node to be used for listening on topics."""

    def __init__(self, name='test_node'):
        super().__init__(node_name=name)
        self.msg_event_object = Event()
        self.logger = rclpy.logging.get_logger('wait_for_topics')

    def start_subscribers(self, topic_tuples):
        self.subscriber_list = []
        self.expected_topics = [name for name, _ in topic_tuples]
        self.expected_topics.sort()
        self.received_topics = []
        self.topic_mutex = Lock()

        for topic_name, topic_type in topic_tuples:
            # Create a subscriber
            self.subscriber_list.append(
                self.create_subscription(
                    topic_type,
                    topic_name,
                    self.callback_template(topic_name),
                    10
                )
            )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def callback_template(self, topic_name):

        def topic_callback(data):
            self.topic_mutex.acquire()
            if topic_name not in self.received_topics:
                self.logger.info('Message received for ' + topic_name)
                self.received_topics.append(topic_name)
                self.received_topics.sort()
            if self.received_topics == self.expected_topics:
                self.msg_event_object.set()

            self.topic_mutex.release()

        return topic_callback
