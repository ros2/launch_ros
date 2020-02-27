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

from launch import LaunchContext

from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count


def test_node_name_count_basic():
    lc = LaunchContext()
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_2')
    assert get_node_name_count(lc, 'node_name_1') == 2
    assert get_node_name_count(lc, 'node_name_2') == 1
    assert get_node_name_count(lc, 'node_name_3') == 0
