# Copyright 2025 Open Source Robotics Foundation, Inc.
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


class Repeater(Node):

    def __init__(self):
        super().__init__('repeater')
        self.subscription = self.create_subscription(
            String, 'input', self.callback, 10
        )
        self.publisher = self.create_publisher(String, 'output', 10)

    def callback(self, input_msg):
        self.get_logger().info(f'I heard: [{input_msg.data}]')
        output_msg_data = input_msg.data
        self.get_logger().info(f'Publishing: "{output_msg_data}"')
        self.publisher.publish(String(data=output_msg_data))


def main(args=None):
    rclpy.init(args=args)

    node = Repeater()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
