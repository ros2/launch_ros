# Copyright 2026 Dexory
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

"""
Regression test for the isolated component_container load race.

Starts several isolated ``component_container`` processes concurrently, each
loading a composable node. The load clients are already waiting when their
container starts, which exercises the window where the isolated container path
briefly advertises a throwaway ``ComponentManager`` (owning
``~/_container/load_node``) while reading the ``thread_num`` parameter and then
destroys it before creating the real manager. A load request arriving during
that window must not be dropped: every composable node is expected to appear.
"""

import asyncio

from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.utilities import get_node_name_count

import osrf_pycommon.process_utils

# Several containers started at once make the startup race easy to hit.
NUM_CONTAINERS = 8


def _assert_launch_no_errors(actions, *, timeout_sec=15):
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


def test_isolated_container_load_race():
    """All composables must load across several concurrent isolated containers."""
    actions = []
    for i in range(NUM_CONTAINERS):
        actions.append(
            ComposableNodeContainer(
                package='rclcpp_components',
                executable='component_container',
                arguments=['--executor-type', 'single-threaded', '--isolated'],
                name=f'test_isolated_container_{i}',
                namespace='',
                composable_node_descriptions=[
                    ComposableNode(
                        package='composition',
                        plugin='composition::Listener',
                        name=f'loaded_component_{i}',
                        namespace='',
                    )
                ],
            )
        )

    context = _assert_launch_no_errors(actions)

    for i in range(NUM_CONTAINERS):
        assert get_node_name_count(context, f'/test_isolated_container_{i}') == 1
        assert get_node_name_count(context, f'/loaded_component_{i}') == 1
