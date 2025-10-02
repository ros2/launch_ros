# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the LifecycleNode action."""

from typing import List
from typing import Optional

import launch
from launch import SomeSubstitutionsType
from launch.action import Action
import launch.logging

import lifecycle_msgs.msg
import lifecycle_msgs.srv

from .lifecycle_transition import LifecycleTransition
from .node import Node

from ..utilities import LifecycleEventManager


class LifecycleNode(Node):
    """Action that executes a ROS lifecycle node."""

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        autostart: bool = False,
        **kwargs
    ) -> None:
        """
        Construct a LifecycleNode action.

        Almost all of the arguments are passed to :class:`Node` and eventually
        to :class:`launch.actions.ExecuteProcess`, so see the documentation of
        those classes for additional details.

        This action additionally emits some event(s) in certain circumstances:

        - :class:`launch.events.lifecycle.StateTransition`:

            - this event is emitted when a message is published to the
              "/<name>/transition_event" topic, indicating the lifecycle
              node represented by this action changed state

        This action also handles some events related to lifecycle:

        - :class:`launch.events.lifecycle.ChangeState`

          - this event can be targeted to a single lifecycle node, or more than
            one, or even all lifecycle nodes, and it requests the targeted nodes
            to change state, see its documentation for more details.

        :param name: The name of the lifecycle node.
          Although it defaults to None it is a required parameter and the default will be removed
          in a future release.
        :param namespace: The ROS namespace for this Node
        :param autostart: Whether or not to automatically transition to the 'active' state.
        """
        super().__init__(name=name, namespace=namespace, **kwargs)
        self.__logger = launch.logging.get_logger(__name__)
        self.__autostart = autostart
        self.__lifecycle_event_manager = None

    @property
    def node_autostart(self):
        """Getter for autostart."""
        return self.__autostart

    @property
    def is_lifecycle_node(self):
        return True

    def get_lifecycle_event_manager(self):
        return self.__lifecycle_event_manager

    def execute(self, context: launch.LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)  # ensure self.node_name is expanded
        if '<node_name_unspecified>' in self.node_name:
            raise RuntimeError('node_name unexpectedly incomplete for lifecycle node')

        self.__lifecycle_event_manager = LifecycleEventManager(self)
        self.__lifecycle_event_manager.setup_lifecycle_manager(context)

        # If autostart is enabled, transition to the 'active' state.
        autostart_actions = None
        if self.node_autostart:
            autostart_actions = [
                LifecycleTransition(
                    lifecycle_node_names=[self.node_name],
                    transition_ids=[lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                                    lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE]
                ),
            ]

        # Delegate execution to Node and ExecuteProcess.
        node_actions = super().execute(context)  # type: Optional[List[Action]]
        if node_actions is not None and autostart_actions is not None:
            return node_actions + autostart_actions
        if node_actions is not None:
            return node_actions
        if autostart_actions is not None:
            return autostart_actions
        return None
