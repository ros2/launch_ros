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

import os
import unittest

from composition_interfaces.srv import LoadNode
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros import get_default_launch_description
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.asserts
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType


def generate_test_description(ready_fn):
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    launch_description = LaunchDescription()

    mock_container = ComposableNodeContainer(
        env=proc_env,
        node_name='my_container',
        node_namespace='/my_ns',
        package='test_launch_ros',
        node_executable='mock-composable-container',
        composable_node_descriptions=[
            ComposableNode(package='fake_package', node_plugin='successfully_load'),
            ComposableNode(package='fake_package', node_plugin='fail_to_load'),
            ComposableNode(
                package='fake_package', node_plugin='node_name',
                node_name='my_talker'
            ),
            ComposableNode(
                package='fake_package', node_plugin='node_namespace',
                node_namespace='my_namespace'
            ),
            ComposableNode(
                package='fake_package', node_plugin='remap_rules',
                remappings=[('~/foo', '/bar')]
            ),
            ComposableNode(
                package='fake_package', node_plugin='parameters',
                parameters=[{'foo': {'bar': 'baz'}}]
            ),
            ComposableNode(
                package='fake_package', node_plugin='extra_arguments',
                extra_arguments=[{'ping.pong': 5}]
            ),
            # TODO(sloretz) log level
            # ComposableNode(
            #     package='fake_package', node_plugin='log_level',
            #     log_level=1
            # ),
        ])

    launch_description.add_action(get_default_launch_description())
    launch_description.add_action(mock_container)
    # TODO(sloretz) post-launch composable node actions
    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=mock_container,
    #             on_start=[
    #                 LoadComposableNodes(
    #                     composable_node_descriptions=[
    #                         ComposableNode(
    #                             package='fake_package', node_plugin='node_name',
    #                             node_name='my_talker'
    #                         ),
    #                     ],
    #                     target_container=mock_container
    #                 )
    #             ]
    #         )
    #     )
    # )
    launch_description.add_action(
        OpaqueFunction(function=lambda context: ready_fn())
    )

    return launch_description, {'container': mock_container}


class TestComposition(unittest.TestCase):

    def test_successfully_load(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'successfully_load'
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_fail_to_load(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'fail_to_load'
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_custom_node_name(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'node_name'
        request.node_name = 'my_talker'
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_custom_node_namespace(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'node_namespace'
        request.node_namespace = 'my_namespace'
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_custom_remap_rules(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'remap_rules'
        request.remap_rules = ['~/foo:=/bar']
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_custom_parameters(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'parameters'
        p = Parameter()
        p.name = 'foo.bar'
        p.value.string_value = 'baz'
        p.value.type = ParameterType.PARAMETER_STRING
        request.parameters = [p]
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    def test_custom_extra_arguments(self, container):
        request = LoadNode.Request()
        request.package_name = 'fake_package'
        request.plugin_name = 'extra_arguments'
        p = Parameter()
        p.name = 'ping.pong'
        p.value.integer_value = 5
        p.value.type = ParameterType.PARAMETER_INTEGER
        request.extra_arguments = [p]
        self.proc_output.assertWaitFor(expected_output=repr(request), process=container)

    # TODO(sloretz) log level
    # def test_custom_log_level(self, container):
    #     request = LoadNode.Request()
    #     request.package_name = 'fake_package'
    #     request.plugin_name = 'log_level'
    #     request.log_level = 1
    #     self.proc_output.assertWaitFor(expected_output=repr(request), process=container)
