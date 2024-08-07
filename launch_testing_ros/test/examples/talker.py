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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.count = 0
        hz_param = self.declare_parameter('hz', 1.0)
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0 / hz_param.value, self.callback)

    def callback(self):
        data = 'Hello World: {0}'.format(self.count)
        self.get_logger().info('Publishing: "{0}"'.format(data))
        self.publisher.publish(String(data=data))
        self.count += 1


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Talker()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
