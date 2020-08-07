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

import asyncio

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.utilities import get_node_name_count

import osrf_pycommon.process_utils

TEST_CONTAINER_NAME = 'test_load_composable_nodes_container'
TEST_NODE_NAME = 'test_load_composable_nodes_node'


def _assert_launch_no_errors(actions, *, timeout_sec=1):
    ld = LaunchDescription(actions)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)

    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())
    loop.run_until_complete(asyncio.sleep(timeout_sec))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    assert 0 == launch_task.result()
    return ls.context


def _create_node_container(*, parameters=None, remappings=None, namespace=''):
    return ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name=TEST_CONTAINER_NAME,
        namespace=namespace,
        output='screen',
        parameters=parameters,
        remappings=remappings,
    )


def _load_composable_node(
    *,
    parameters=None,
    remappings=None,
    target_container=TEST_CONTAINER_NAME
):
    return LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name=TEST_NODE_NAME,
                parameters=parameters,
                remappings=remappings,
            )
        ])


def test_load_invalid_node():
    """Test loading an invalid node."""
    load_composable_nodes = LoadComposableNodes(
        target_container=TEST_CONTAINER_NAME,
        composable_node_descriptions=[
            ComposableNode(
                package='nonexistent_package',
                plugin='this_plugin_should_not_exist',
                name=TEST_NODE_NAME,
            )
        ])
    context = _assert_launch_no_errors([_create_node_container(), load_composable_nodes])

    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 0
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAME}') == 1


def test_load_node():
    """Test loading a node."""
    context = _assert_launch_no_errors([
        _create_node_container(), _load_composable_node()
    ])

    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAME}') == 1


def test_load_node_with_global_remaps_in_group():
    """Test loading a node with global remaps scoped to a group."""
    load_composable_nodes_action = _load_composable_node()
    context = _assert_launch_no_errors([
        _create_node_container(),
        GroupAction(
            [
                SetRemap('chatter', 'new_topic_name'),
                load_composable_nodes_action,
            ],
            scoped=True,
        ),
    ])

    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAME}') == 1

    # Check the remaps in load service request
    assert len(load_composable_nodes_action._LoadComposableNodes__load_node_requests) == 1
    request = load_composable_nodes_action._LoadComposableNodes__load_node_requests[0]
    assert len(request.remap_rules) == 1
    assert request.remap_rules[0] == 'chatter:=new_topic_name'


def test_load_node_with_namespace_in_group():
    """Test loading a node with namespace scoped to a group."""
    namespace = '/foo'
    load_composable_nodes_action = _load_composable_node()
    context = _assert_launch_no_errors([
        _create_node_container(),
        GroupAction(
            [
                PushRosNamespace(namespace),
                load_composable_nodes_action,
            ],
            scoped=True,
        ),
    ])

    assert get_node_name_count(context, f'{namespace}/{TEST_NODE_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAME}') == 1

    # Check the namespace in load service request
    assert len(load_composable_nodes_action._LoadComposableNodes__load_node_requests) == 1
    request = load_composable_nodes_action._LoadComposableNodes__load_node_requests[0]
    assert request.node_namespace == f'{namespace}'
