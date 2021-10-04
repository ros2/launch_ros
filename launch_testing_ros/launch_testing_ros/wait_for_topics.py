from threading import Event
from threading import Thread

import rclpy
from rclpy.node import Node

class WaitForTopics:
    def __init__(self, topic_name, topic_type):
        self.__ros_context = None
        self.__ros_node = None
        self.topic_name = topic_name
        self.topic_type = topic_type

        self.start()

    def start(self):
        self.__ros_context = rclpy.Context()
        rclpy.init()
        self.__ros_node = MakeTestNode(name='test_node')

    def wait(self, timeout=5.0):
        self.__ros_node.start_subscriber(self.topic_name, self.topic_type)
        return self.__ros_node.msg_event_object.wait(timeout)

    def shutdown(self):
        rclpy.shutdown()


class MakeTestNode(Node):
    def __init__(self, name='test_node'):
        super().__init__(node_name=name)
        self.msg_event_object = Event()

    def start_subscriber(self, topic_name, topic_type):
        # Create a subscriber
        self.subscription = self.create_subscription(
            topic_type,
            topic_name,
            self.subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_event_object.set()
