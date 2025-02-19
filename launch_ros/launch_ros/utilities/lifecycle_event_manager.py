# Copyright 2024 Open Navigation LLC
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

"""Module for the management of LifecycleNode events."""

import functools
import threading
from typing import cast

import launch
import launch.logging

import lifecycle_msgs.msg
import lifecycle_msgs.srv

from ..events.lifecycle import ChangeState
from ..events.lifecycle import StateTransition

from ..ros_adapters import get_ros_node


class LifecycleEventManager:

    def __init__(self, lifecycle_node) -> None:
        """
        Construct a LifecycleEventManager utility.

        This utility emits some event(s) in certain circumstances:

        - :class:`launch.events.lifecycle.StateTransition`:

            - this event is emitted when a message is published to the
              "/<name>/transition_event" topic, indicating the lifecycle
              node represented by this action changed state

        This utility also handles some events related to lifecycle:

        - :class:`launch.events.lifecycle.ChangeState`

          - this event can be targeted to a single lifecycle node, or more than
            one, or even all lifecycle nodes, and it requests the targeted nodes
            to change state, see its documentation for more details.

        :param lifecycle_node: The lifecycle node to event manage.
        """
        self.__logger = launch.logging.get_logger(__name__)
        self.__lifecycle_node = lifecycle_node
        self.__rclpy_subscription = None
        self.__rclpy_change_state_client = None

    @property
    def node_name(self):
        return self.__lifecycle_node.node_name

    def _on_transition_event(self, context, msg):
        try:
            event = StateTransition(action=self.__lifecycle_node, msg=msg)
            context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))
        except Exception as exc:
            self.__logger.error(
                "Exception in handling of 'lifecycle.msg.TransitionEvent': {}".format(exc))

    def _call_change_state(self, request, context: launch.LaunchContext):
        while not self.__rclpy_change_state_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                return

        # Asynchronously wait so that we can periodically check for shutdown.
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        response_future = self.__rclpy_change_state_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                response_future.cancel()
                return

        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        if not response.success:
            self.__logger.error(
                "Failed to make transition '{}' for LifecycleNode '{}'".format(
                    ChangeState.valid_transitions[request.transition.id],
                    self.node_name,
                )
            )

    def _on_change_state_event(self, context: launch.LaunchContext) -> None:
        typed_event = cast(ChangeState, context.locals.event)
        if not typed_event.lifecycle_node_matcher(self.__lifecycle_node):
            return None
        request = lifecycle_msgs.srv.ChangeState.Request()
        request.transition.id = typed_event.transition_id
        context.add_completion_future(
            context.asyncio_loop.run_in_executor(None, self._call_change_state, request, context))

    def setup_lifecycle_manager(self, context: launch.LaunchContext) -> None:
        node = get_ros_node(context)

        # Create a subscription to monitor the state changes of the subprocess.
        self.__rclpy_subscription = node.create_subscription(
            lifecycle_msgs.msg.TransitionEvent,
            '{}/transition_event'.format(self.node_name),
            functools.partial(self._on_transition_event, context),
            10)

        # Create a service client to change state on demand.
        self.__rclpy_change_state_client = node.create_client(
            lifecycle_msgs.srv.ChangeState,
            '{}/change_state'.format(self.node_name))

        # Register an event handler to change states on a ChangeState lifecycle event.
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeState),
            entities=[launch.actions.OpaqueFunction(function=self._on_change_state_event)],
        ))
