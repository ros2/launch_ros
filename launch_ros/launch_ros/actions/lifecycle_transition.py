# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from typing import Text
from typing import Optional
from typing import List
from collections import OrderedDict
import launch
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.action import Action
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, UnregisterEventHandler

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState, StateTransition
from lifecycle_msgs.msg import Transition
from launch_ros.events.matchers import matches_node_name


@expose_action('lifecycle_transition')
class LifecycleTransition(Action):
    """An action that simplifies execution of lifecyle transitions."""

    transition_targets = OrderedDict([
        (
            Transition.TRANSITION_CONFIGURE,
            {'start_state': 'configuring', 'goal_state': 'inactive'},
        ),
        (
            Transition.TRANSITION_CLEANUP,
            {'start_state': 'cleaningup', 'goal_state': 'unconfigured'},
        ),
        (
            Transition.TRANSITION_ACTIVATE,
            {'start_state': 'activating', 'goal_state': 'active'},
        ),
        (
            Transition.TRANSITION_DEACTIVATE,
            {'start_state': 'deactivating', 'goal_state': 'inactive'},
        ),
        (
            Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
            {'start_state': 'shuttingdown', 'goal_state': 'finalized'},
        ),
        (
            Transition.TRANSITION_INACTIVE_SHUTDOWN,
            {'start_state': 'shuttingdown', 'goal_state': 'finalized'},
        ),
        (
            Transition.TRANSITION_ACTIVE_SHUTDOWN,
            {'start_state': 'shuttingdown', 'goal_state': 'finalized'},
        ),
    ])

    def __init__(
            self,
            *,
            lifecycle_node_names: List[Text],
            transitions_ids: List[int],
            **kwargs, ) -> None:
        """
        Construct a LifecycleTransition action.

        The action will execute the passed in lifecycle transition for the
        lifecycle nodes with the indicated node names. The action will emit
        an event that triggers the first lifecycle transition of each node
        wait that the node reaches the transition goal and trigger the next
        transition in the list.
        You need to make sure, that the sequence of lifecyle transition you
        pass in is possible.

        :param lifecycle_node_names: The names of the lifecycle nodes to transition
        :param transitions_ids: The transitions to be executed.
        """
        super().__init__(**kwargs)
        self.__lifecycle_node_names = lifecycle_node_names
        self.__transition_ids = transitions_ids

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse load_composable_node."""
        _, kwargs = super().parse(entity, parser)

        kwargs['lifecycle_node_names'] = entity.get_attr(
            'lifecycle_node_names', data_type=List[str])
        # Probably need to make this parsable through a dict for convenience
        kwargs['transitions_ids'] = entity.get_attr(
            'transitions_ids', data_type=List[int])
        return cls, kwargs

    def execute(
        self,
        context: launch.LaunchContext
    ) -> Optional[List[Action]]:
        """
        Execute the Lifecycle Transition action.

        :return Returns a list of actions to be executed to achieve specified transitions.
          These are EventHandlers and EventEmitters for ChangeState and
          StateTransition events of the nodes indicated.
        """
        emit_actions = OrderedDict()
        event_handlers = {}
        actions: List[Action] = []

        # Create EmitEvents for ChangeStates and store
        for name in self.__lifecycle_node_names:
            own_emit_actions = []
            for id in self.__transition_ids:
                change_event = ChangeState(
                    lifecycle_node_matcher=matches_node_name(name),
                    transition_id=id)
                emit_action = EmitEvent(
                    event=change_event
                )
                own_emit_actions.append(emit_action)
            emit_actions[name] = own_emit_actions
            event_handlers[name] = []
        # Create Transition EventHandlers and Registration actions
        i = 1
        for id in self.__transition_ids:
            # Create Transition handler for all indicated nodes
            for node_name in self.__lifecycle_node_names:

                states = self.transition_targets[id]
                event_handler = None
                # For all transitions except the last, emit next ChangeState Event
                if i < len(self.__transition_ids):
                    event_handler = OnStateTransition(
                        matcher=match_node_name_goal(
                            node_name,
                            states["start_state"],
                            states["goal_state"]),
                        entities=[
                            emit_actions[node_name][i]],
                        handle_once=True
                    )
                # For last transition emit Log message
                else:
                    event_handler = OnStateTransition(
                        matcher=match_node_name_goal(
                            node_name,
                            states["start_state"],
                            states["goal_state"]),
                        entities=[
                            LogInfo(
                                msg="Tranistioning done, reached {} for node {}".format(
                                    states["goal_state"], node_name)
                            )
                        ],
                        handle_once=True
                    )
                event_handlers[node_name].append(event_handler)
                # Create register event handler action
                register_action = RegisterEventHandler(event_handler=event_handler)
                # Append to actions
                actions.append(register_action)
            # increment next ChangeState action by one
            i += 1

        # Remove consequent transitions if error occurs
        for node_name in self.__lifecycle_node_names:
            unregister_actions = []
            for event_handler in event_handlers[node_name]:
                unregister_actions.append(UnregisterEventHandler(event_handler))
            unregister_actions.append(
                LogInfo(msg="Transition for {} failed. Unregistering all event based \
                transitions for this node.".format(node_name)))
            event_handler = OnStateTransition(
                matcher=match_node_name_goal(node_name, 'errorprocessing', 'unconfigured'),
                entities=unregister_actions,
                handle_once=True)

        # Add first Emit actions to actions
        for node_name in self.__lifecycle_node_names:
            actions.append(emit_actions[node_name][0])

        return actions


def match_node_name_goal(node_name: str, start_state: str, goal_state: str):
    if not node_name.startswith('/'):
        node_name = f'/{node_name}'
    return lambda event: (
        isinstance(event, StateTransition) and
        (event.action.node_name == node_name) and
        (event.goal_state == goal_state) and
        (event.start_state == start_state)
        )
