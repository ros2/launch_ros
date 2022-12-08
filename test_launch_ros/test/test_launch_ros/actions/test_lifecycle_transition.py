# Copyright 2022 Christoph Hellmann Santos
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

"""Tests for the LifcycleTransition action."""

from launch import LaunchContext
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition

import pytest


def test_lifecycle_transition_constructor():
    LifecycleTransition(
        lifecycle_node_names=['talker'],
        transition_ids=[Transition.TRANSITION_ACTIVATE]
    )
    LifecycleTransition(
        lifecycle_node_names=['talker'],
        transition_ids=[1]
    )
    LifecycleTransition(
        lifecycle_node_names=['talker'],
        transition_ids=['1']
    )
    with pytest.raises(ValueError):
        LifecycleTransition(
            lifecycle_node_names=['talker'],
            transition_ids=[]
        )
    with pytest.raises(ValueError):
        LifecycleTransition(
            lifecycle_node_names=[],
            transition_ids=[Transition.TRANSITION_ACTIVATE]
        )


def test_lifecycle_transition_execute():
    lc = LaunchContext()
    lt = LifecycleTransition(
        lifecycle_node_names=['talker'],
        transition_ids=['1']
    )
    actions = lt.execute(lc)
    # Check that actions are correctly generated
    # First action should be RegisterEventHandler for first Transtion
    assert isinstance(actions[0], RegisterEventHandler)
    # Second action should be EmitEvent for first Transition
    assert isinstance(actions[1], EmitEvent)
