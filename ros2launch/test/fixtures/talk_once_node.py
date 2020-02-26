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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TalkOncePublisher(Node):

    def __init__(self):
        super().__init__('talk_once_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.count
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    talk_once_publisher = TalkOncePublisher()

    while rclpy.ok():
        rclpy.spin_once(talk_once_publisher)
        if talk_once_publisher.count > 0:
            break

    talk_once_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
