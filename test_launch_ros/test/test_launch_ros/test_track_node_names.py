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

"""Tests for the node name count tracking utility."""

import asyncio

import osrf_pycommon.process_utils
import rclpy

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService

from launch_ros import get_default_launch_description
from launch_ros.actions.node import Node
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.descriptions.composable_node import ComposableNode
from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count


TEST_NODE_NAME = 'my_node'


def test_node_name_count():
    lc = LaunchContext()
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_2')
    assert get_node_name_count(lc, 'node_name_1') == 2
    assert get_node_name_count(lc, 'node_name_2') == 1
    assert get_node_name_count(lc, 'node_name_3') == 0


def _launch(launch_description):
    loop = osrf_pycommon.process_utils.get_loop()
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(launch_description)
    launch_task = loop.create_task(ls.run_async())
    loop.run_until_complete(asyncio.sleep(5, loop=loop))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    rclpy.shutdown()
    print(ls.context.locals.unique_ros_node_names)
    return ls.context


def test_launch_node_with_name():
    node = Node(
        package='demo_nodes_py',
        node_executable='listener_qos',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        output='screen',
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, '/{}'.format(TEST_NODE_NAME)) == 1


def test_launch_node_without_name():
    node = Node(
        package='demo_nodes_py',
        node_executable='listener_qos',
        node_namespace='',
        output='screen',
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, None) == 1


def test_launch_composable_node_with_names():
    node = ComposableNodeContainer(
        package='rclcpp_components',
        node_executable='component_container',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                node_plugin='composition::Listener',
                node_name=TEST_NODE_NAME
            )
        ],
        output='screen'
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, '/{}'.format(TEST_NODE_NAME)) == 2


def test_launch_composable_node_without_component_name():
    node = ComposableNodeContainer(
        package='rclcpp_components',
        node_executable='component_container',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                node_plugin='composition::Listener',
            )
        ],
        output='screen'
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, '/{}'.format(TEST_NODE_NAME)) == 1
    assert get_node_name_count(context, '/listener') == 1


def test_launch_nodes_with_same_names():
    node1 = Node(
        package='demo_nodes_py',
        node_executable='listener_qos',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        output='screen',
    )

    node2 = LifecycleNode(
        package='lifecycle',
        node_executable='lifecycle_listener',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        output='screen',
    )

    node3 = ComposableNodeContainer(
        package='rclcpp_components',
        node_executable='component_container',
        node_name=TEST_NODE_NAME,
        node_namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                node_plugin='composition::Listener',
                node_name=TEST_NODE_NAME
            )
        ],
        output='screen'
    )

    ld = LaunchDescription([node1, node2, node3])
    context = _launch(ld)
    assert get_node_name_count(context, '/{}'.format(TEST_NODE_NAME)) == 4
