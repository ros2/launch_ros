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

"""Tests for the PushRosNamespace Action."""

from launch import LaunchContext

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import pytest


class Config:

    def __init__(
        self,
        *,
        node_ns='',
        push_ns=None,
        expected_ns=None,
        second_push_ns=None
    ):
        self.push_ns = push_ns
        self.node_ns = node_ns
        self.expected_ns = expected_ns
        self.second_push_ns = second_push_ns


@pytest.mark.parametrize('config', (
    Config(
        push_ns='relative_ns',
        node_ns='node_ns',
        expected_ns='/relative_ns/node_ns'),
    Config(
        push_ns='relative_ns',
        node_ns='/node_ns',
        expected_ns='/node_ns'),
    Config(
        push_ns='relative_ns',
        node_ns='/',
        expected_ns='/'),
    Config(
        push_ns='relative_ns',
        node_ns='',
        expected_ns='/relative_ns'),
    Config(
        push_ns='relative_ns',
        second_push_ns='another_relative_ns',
        node_ns='node_ns',
        expected_ns='/relative_ns/another_relative_ns/node_ns'),
    Config(
        push_ns='relative_ns',
        second_push_ns='/absolute_ns',
        node_ns='node_ns',
        expected_ns='/absolute_ns/node_ns'),
    Config(),
    Config(push_ns=''),
))
def test_push_ros_namespace(config):
    lc = LaunchContext()
    if config.push_ns is not None:
        pns1 = PushRosNamespace(config.push_ns)
        pns1.execute(lc)
    if config.second_push_ns is not None:
        pns2 = PushRosNamespace(config.second_push_ns)
        pns2.execute(lc)
    node = Node(
        package='dont_care',
        executable='whatever',
        namespace=config.node_ns,
    )
    node._perform_substitutions(lc)
    expected_ns = config.expected_ns if config.expected_ns is not None else ''
    assert expected_ns == node.expanded_node_namespace
