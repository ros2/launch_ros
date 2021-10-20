# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests for the LoadComposableNodes Action."""

import threading

from composition_interfaces.srv import LoadNode

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import GroupAction
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.utilities import get_node_name_count

import pytest

from rcl_interfaces.msg import ParameterType

import rclpy
import rclpy.context
import rclpy.executors
import rclpy.node

TEST_CONTAINER_NAME = 'mock_component_container'
TEST_NODE_NAME = 'test_load_composable_nodes_node'


class MockComponentContainer(rclpy.node.Node):

    def __init__(self):
        # List of LoadNode requests received
        self.requests = []

        self._context = rclpy.context.Context()
        rclpy.init(context=self._context)

        super().__init__(TEST_CONTAINER_NAME, context=self._context)

        self.load_node_service = self.create_service(
            LoadNode,
            '~/_container/load_node',
            self.load_node_callback
        )

        self._executor = rclpy.executors.SingleThreadedExecutor(context=self._context)

        # Start spinning in a thread
        self._thread = threading.Thread(
            target=rclpy.spin,
            args=(self, self._executor),
            daemon=True
        )
        self._thread.start()

    def load_node_callback(self, request, response):
        self.requests.append(request)
        response.success = True
        response.full_node_name = f'{request.node_namespace}/{request.node_name}'
        response.unique_id = len(self.requests)
        return response

    def shutdown(self):
        self._executor.shutdown()
        rclpy.shutdown(context=self._context)
        self.destroy_node()
        self._thread.join()


def _assert_launch_no_errors(actions):
    ld = LaunchDescription(actions)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    return ls.context


def _load_composable_node(
    *,
    package,
    plugin,
    name,
    namespace='',
    parameters=None,
    remappings=None,
    target_container=f'/{TEST_CONTAINER_NAME}'
):
    return LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            ComposableNode(
                package=package,
                plugin=plugin,
                name=name,
                namespace=namespace,
                parameters=parameters,
                remappings=remappings,
            )
        ])


@pytest.fixture
def mock_component_container():
    container = MockComponentContainer()
    yield container
    container.shutdown()


def test_load_node(mock_component_container):
    """Test loading a node."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace'
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container recieved correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_remaps(mock_component_container):
    """Test loading a node with remappings."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace',
            remappings=[
                ('test_topic1', 'test_remap_topic1'),
                ('test/topic/two', 'test/remap_topic2')
            ]
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container recieved correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 2
    assert request.remap_rules[0] == 'test_topic1:=test_remap_topic1'
    assert request.remap_rules[1] == 'test/topic/two:=test/remap_topic2'
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_params(mock_component_container):
    """Test loading a node with parameters."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace',
            parameters=[{
                'test_param1': 'test_value_param1',
                'test.param2': '42.0',
            }]
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container recieved correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'test_param1'
    assert request.parameters[0].value.type == ParameterType.PARAMETER_STRING
    assert request.parameters[0].value.string_value == 'test_value_param1'
    assert request.parameters[1].name == 'test.param2'
    # TODO(jacobperron): I would expect this to be a double value, but we can only pass strings
    # assert request.parameters[1].value.type == ParameterType.PARAMETER_DOUBLE
    # assert request.parameters[1].value.double_value == 42.0
    assert request.parameters[1].value.string_value == '42.0'
    assert len(request.extra_arguments) == 0


def test_load_node_with_global_remaps_in_group(mock_component_container):
    """Test loading a node with global remaps scoped to a group."""
    context = _assert_launch_no_errors([
        GroupAction(
            [
                SetRemap('chatter', 'new_topic_name'),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name',
                    namespace='test_node_namespace'
                ),
            ],
            scoped=True,
        ),
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container recieved correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 1
    assert request.remap_rules[0] == 'chatter:=new_topic_name'
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_namespace_in_group(mock_component_container):
    """Test loading a node with namespace scoped to a group."""
    context = _assert_launch_no_errors([
        GroupAction(
            [
                PushRosNamespace('foo'),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name',
                    namespace='test_node_namespace'
                ),
            ],
            scoped=True,
        ),
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/foo/test_node_namespace/test_node_name') == 1

    # Check that container recieved correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/foo/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0
