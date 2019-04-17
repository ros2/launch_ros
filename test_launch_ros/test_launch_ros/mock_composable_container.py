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

import sys

from composition_interfaces.srv import LoadNode
import rclpy
from rclpy.node import Node


class MockComposableNodeContainer(Node):

    def __enter__(self):
        self.__load_service = self.create_service(
            LoadNode, '~/_container/load_node', self._on_load_node)
        return self

    def __exit__(self, type_, value, traceback):
        self.destroy_service(self.__load_service)
        self.destroy_node()

    def __init__(self, name, namespace):
        """
        Initialize a mock container process.

        :param load_node_responses: responses to a load node request for a given package/plugin
        :type load_node_responses: {(str, str): composition_interfaces.LoadNode.Response, ...}
        """
        super().__init__(name, namespace=namespace)

        # Canned responses to load service
        fail_to_load = LoadNode.Response()
        fail_to_load.success = False
        fail_to_load.error_message = 'intentional failure'

        successfully_load = LoadNode.Response()
        successfully_load.success = True
        successfully_load.full_node_name = '/a_nodename'
        successfully_load.unique_id = 2

        node_name = LoadNode.Response()
        node_name.success = True
        node_name.full_node_name = '/my_talker'
        node_name.unique_id = 4

        node_namespace = LoadNode.Response()
        node_namespace.success = True
        node_namespace.full_node_name = '/my_namespace/my_talker'
        node_namespace.unique_id = 8

        log_level = LoadNode.Response()
        log_level.success = True
        log_level.full_node_name = '/a_nodename'
        log_level.unique_id = 16

        remap_rules = LoadNode.Response()
        remap_rules.success = True
        remap_rules.full_node_name = '/a_nodename'
        remap_rules.unique_id = 32

        parameters = LoadNode.Response()
        parameters.success = True
        parameters.full_node_name = '/a_nodename'
        parameters.unique_id = 64

        extra_arguments = LoadNode.Response()
        extra_arguments.success = True
        extra_arguments.full_node_name = '/a_nodename'
        extra_arguments.unique_id = 128

        node_name_on_event = LoadNode.Response()
        node_name_on_event.success = True
        node_name_on_event.full_node_name = '/my_talker_on_event'
        node_name_on_event.unique_id = 256

        self.__load_node_responses = {
            ('fake_package', 'fail_to_load'): fail_to_load,
            ('fake_package', 'successfully_load'): successfully_load,
            ('fake_package', 'node_name'): node_name,
            ('fake_package', 'node_namespace'): node_namespace,
            ('fake_package', 'log_level'): log_level,
            ('fake_package', 'remap_rules'): remap_rules,
            ('fake_package', 'parameters'): parameters,
            ('fake_package', 'extra_arguments'): extra_arguments,
            ('fake_package', 'node_name_on_event'): node_name_on_event,
        }

        self.unexpected_request = False

    def _on_load_node(self, request, response):
        key = (request.package_name, request.plugin_name)
        if key not in self.__load_node_responses:
            self.unexpected_request = True
            unexpected_load = LoadNode.Response()
            unexpected_load.success = False
            unexpected_load.error_message = 'unexpected load request'
            return response
        else:
            print(repr(request))
            return self.__load_node_responses[key]
        return response


def main():
    rclpy.init()
    container = MockComposableNodeContainer(name='mock_container', namespace='/')
    with container:
        try:
            rclpy.spin(container)
        except KeyboardInterrupt:
            print('Got SIGINT, shutting down')
        except:
            import traceback
            traceback.print_exc()
    if container.unexpected_request:
        sys.stderr.write('failing due to unexpected request\n')
        sys.exit(1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
