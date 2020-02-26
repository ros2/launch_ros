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


class ListenOnceSubscriber(Node):

    def __init__(self):
        super().__init__('listen_once_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.count = 0

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    listen_once_subscriber = ListenOnceSubscriber()

    while rclpy.ok():
        rclpy.spin_once(listen_once_subscriber)
        if listen_once_subscriber.count > 0:
            break

    listen_once_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
