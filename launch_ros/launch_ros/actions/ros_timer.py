# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Module for the RosTimer action."""

import asyncio
import collections.abc
from functools import partial
from typing import Any  # noqa: F401
from typing import cast
from typing import Dict  # noqa: F401
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union

from launch.action import Action
from launch.actions.opaque_function import OpaqueFunction

import launch.logging
from launch_ros.ros_adapters import get_ros_node

from launch.event_handler import EventHandler
from launch.events import Shutdown
from launch.events import TimerEvent
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.some_actions_type import SomeActionsType
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.utilities import create_future
from launch.utilities import ensure_argument_type
from launch.utilities import is_a_subclass
from launch.utilities import type_utils


@expose_action('ros_timer')
class RosTimer(Action):
    """
    Action that defers other entities until a period of time has passed, unless canceled.

    This timer uses ROS time instead of wall clock time.
    To enable the use of sim time, you must also use the SetUseSimTime action.
    All timers are "one-shot", in that they only fire one time and never again.
    """

    def __init__(
        self,
        *,
        period: Union[float, SomeSubstitutionsType],
        actions: Iterable[LaunchDescriptionEntity],
        **kwargs
    ) -> None:
        """
        Create a RosTimer.

        :param period: is the time (in seconds) to set the timer for.
        :param actions: is an iterable containing actions to be executed upon on timeout.
        """
        super().__init__(**kwargs)
        period_types = list(SomeSubstitutionsType_types_tuple) + [float]
        ensure_argument_type(period, period_types, 'period', 'RosTimer')
        ensure_argument_type(actions, collections.abc.Iterable, 'actions', 'RosTimer')
        self.__period = type_utils.normalize_typed_substitution(period, float)
        self.__actions = actions
        self.__context_locals: Dict[Text, Any] = {}
        self.__canceled = False
        self.__completed_future: Optional[asyncio.Future] = None
        self.__canceled_future: Optional[asyncio.Future] = None
        self.__timer_future: Optional[asyncio.Future] = None
        self.__logger = launch.logging.get_logger(__name__)

    def __timer_callback(self):
        if not self.__timer_future.done():
            self.__timer_future.set_result(True)

    async def __wait_to_fire_event(self, context):
        node = get_ros_node(context)
        node.create_timer(
            self.__period,
            partial(context.asyncio_loop.call_soon_threadsafe, self.__timer_callback),
        )

        done, pending = await asyncio.wait(
            [self.__canceled_future, self.__timer_future],
            return_when=asyncio.FIRST_COMPLETED
        )

        if not self.__canceled_future.done():
            await context.emit_event(TimerEvent(timer_action=self))
        self.__completed_future.set_result(None)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Return the `Timer` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['period'] = parser.parse_if_substitutions(
            entity.get_attr('period', data_type=float, can_be_str=True))
        kwargs['actions'] = [parser.parse_action(child) for child in entity.children]
        return cls, kwargs

    @property
    def period(self):
        return self.__period

    @property
    def actions(self):
        return self.__actions

    def describe(self) -> Text:
        """Return a description of this RosTimer."""
        return 'RosTimer(period={}, actions=<actions>)'.format(self.__period)

    def describe_conditional_sub_entities(self) -> List[Tuple[
        Text,
        Iterable['LaunchDescriptionEntity'],
    ]]:
        """Return the actions that will result when the timer expires, but was not canceled."""
        return [('{} seconds pass without being canceled'.format(self.__period), self.__actions)]

    def handle(self, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle firing of timer."""
        context.extend_locals(self.__context_locals)
        return self.__actions

    def cancel(self) -> None:
        """
        Cancel this RosTimer.

        Calling cancel will not fail if the timer has already finished or
        already been canceled or if the timer has not been started yet.
        This function is not thread-safe and should be called only from under
        another coroutine.
        """
        self.__canceled = True
        if self.__canceled_future is not None and not self.__canceled_future.done():
            self.__canceled_future.set_result(True)
        return None

    def execute(self, context: LaunchContext) -> Optional[List['Action']]:
        """
        Execute the action.

        This does the following:
        - register a global event handler for RosTimers if not already done
        - create a task for the coroutine that waits until canceled or timeout
        - coroutine asynchronously fires event after timeout, if not canceled
        """
        self.__completed_future = create_future(context.asyncio_loop)
        self.__timer_future = create_future(context.asyncio_loop)
        self.__canceled_future = create_future(context.asyncio_loop)

        if self.__canceled:
            # In this case, the action was canceled before being executed.
            self.__logger.debug(
                'timer {} not waiting because it was canceled before being executed'.format(self),
            )
            self.__completed_future.set_result(None)
            return None

        # Once per context, install the general purpose OnTimerEvent event handler.
        if not hasattr(context, '_TimerAction__event_handler_has_been_installed'):
            context.register_event_handler(EventHandler(
                matcher=lambda event: is_a_subclass(event, TimerEvent),
                entities=OpaqueFunction(
                    function=lambda context: (
                        cast(TimerEvent, context.locals.event).timer_action.handle(context)
                    )
                ),
            ))
            # This attribute must match the attribute set in the launch TimerAction
            # to ensure only one event handler is created when there are multiple timers.
            setattr(context, '_TimerAction__event_handler_has_been_installed', True)

        # Capture the current context locals so the yielded actions can make use of them too.
        self.__context_locals = dict(context.get_locals_as_dict())  # Capture a copy
        context.asyncio_loop.create_task(self.__wait_to_fire_event(context))

        # The 'shutdown' event will cause timers to cancel so they don't hold up the
        # launch process
        context.register_event_handler(
            EventHandler(
                matcher=lambda event: is_a_subclass(event, Shutdown),
                entities=OpaqueFunction(function=lambda context: self.cancel())
            )
        )

        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return an asyncio Future, used to let the launch system know when we're done."""
        return self.__completed_future
