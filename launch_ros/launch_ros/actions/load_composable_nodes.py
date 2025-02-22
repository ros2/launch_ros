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

"""Module for the LoadComposableNodes action."""

import asyncio
from pathlib import Path
import threading
import os

from typing import List
from typing import Optional
from typing import Text
from typing import Union

import rclpy
import composition_interfaces.srv

from launch.action import Action
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
import launch.logging
from launch.some_entities_type import SomeEntitiesType
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.utilities import ensure_argument_type
from launch.utilities import is_a_subclass
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.parameter_descriptions import ParameterFile

import lifecycle_msgs.msg

from .composable_node_container import ComposableNodeContainer
from .lifecycle_transition import LifecycleTransition

from ..descriptions import ComposableNode
from ..ros_adapters import get_ros_node, ROSAdapter
from ..utilities import add_node_name
from ..utilities import evaluate_parameters
from ..utilities import get_node_name_count
from ..utilities import make_namespace_absolute
from ..utilities import prefix_namespace
from ..utilities import to_parameters_list
from ..utilities.normalize_parameters import normalize_parameter_dict


@expose_action('load_composable_node')
class LoadComposableNodes(Action):
    """Action that loads composable ROS nodes into a running container."""

    def __init__(
        self,
        *,
        composable_node_descriptions: List[ComposableNode],
        target_container: Union[SomeSubstitutionsType, ComposableNodeContainer],
        unload_on_shutdown: Union[bool, SomeSubstitutionsType] = False,
        **kwargs,
    ) -> None:
        """
        Construct a LoadComposableNodes action.

        The container node is expected to provide a `~/_container/load_node` service for
        loading purposes.
        Loading will be performed sequentially.
        When executed, this action will block until the container's load service is available.
        Make sure any LoadComposableNode action is executed only after its container processes
        has started.

        :param composable_node_descriptions: descriptions of composable nodes to be loaded
        :param target_container: the container to load the nodes into
        """
        ensure_argument_type(
            target_container,
            list(SomeSubstitutionsType_types_tuple) +
            [ComposableNodeContainer],
            'target_container',
            'LoadComposableNodes'
        )
        ensure_argument_type(
            unload_on_shutdown,
            list(SomeSubstitutionsType_types_tuple) +
            [bool],
            'unload_on_shutdown',
            'LoadComposableNodes'
        )
        super().__init__(**kwargs)
        self.__composable_node_descriptions = composable_node_descriptions
        self.__target_container = target_container
        self.__unload_on_shutdown = unload_on_shutdown
        self.__final_target_container_name: Optional[Text] = None
        self.__logger = launch.logging.get_logger(__name__)
        self.__loaded_nodes = {}  # Track loaded nodes for unloading
        self.__no_shutdown_future: Optional[asyncio.Future] = None

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse load_composable_node."""
        _, kwargs = super().parse(entity, parser)

        kwargs['target_container'] = parser.parse_substitution(
            entity.get_attr('target', data_type=str))

        composable_nodes = entity.get_attr('composable_node', data_type=List[Entity])
        kwargs['composable_node_descriptions'] = []
        for entity in composable_nodes:
            composable_node_cls, composable_node_kwargs = ComposableNode.parse(parser, entity)
            kwargs['composable_node_descriptions'].append(
                composable_node_cls(**composable_node_kwargs))
        
        kwargs['unload_on_shutdown'] = parser.parse_if_substitutions(
            entity.get_attr('unload_on_shutdown', data_type=bool, optional=True))

        return cls, kwargs

    def _load_node(
        self,
        request: composition_interfaces.srv.LoadNode.Request,
        context: LaunchContext
    ) -> None:
        """
        Load node synchronously.

        :param request: service request to load a node
        :param context: current launch context
        """
        while not self.__rclpy_load_node_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_load_node_client.srv_name
                    )
                )
                return

        # Asynchronously wait on service call so that we can periodically check for shutdown
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        self.__logger.debug(
            "Calling the '{}' service with request '{}'".format(
                self.__rclpy_load_node_client.srv_name, request
            )
        )

        response_future = self.__rclpy_load_node_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_load_node_client.srv_name),
                )
                response_future.cancel()
                return

        # Get response
        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        self.__logger.debug("Received response '{}'".format(response))

        node_name = response.full_node_name if response.full_node_name else request.node_name
        if response.success:
            self.__loaded_nodes[response.unique_id] = node_name
            if node_name is not None:
                add_node_name(context, node_name)
                node_name_count = get_node_name_count(context, node_name)
                if node_name_count > 1:
                    container_logger = launch.logging.get_logger(
                        self.__final_target_container_name
                    )
                    container_logger.warning(
                        'there are now at least {} nodes with the name {} created within this '
                        'launch context'.format(node_name_count, node_name)
                    )
            self.__logger.info("Loaded node '{}' in container '{}'".format(
                response.full_node_name, self.__final_target_container_name
            ))
        else:
            self.__logger.error(
                "Failed to load node '{}' of type '{}' in container '{}': {}".format(
                    node_name, request.plugin_name, self.__final_target_container_name,
                    response.error_message
                )
            )

    def _load_in_sequence(
        self,
        load_node_requests: List[composition_interfaces.srv.LoadNode.Request],
        context: LaunchContext
    ) -> None:
        """
        Load composable nodes sequentially.

        :param load_node_requests: a list of LoadNode service requests to execute
        :param context: current launch context
        """
        next_load_node_request = load_node_requests[0]
        load_node_requests = load_node_requests[1:]
        self._load_node(next_load_node_request, context)
        if len(load_node_requests) > 0:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._load_in_sequence, load_node_requests, context
                )
            )

    def _unload_node(
        self,
        request: composition_interfaces.srv.UnloadNode.Request,
        client: rclpy.client.Client
    ) -> None:
        """
        Unload node synchronously.

        :param request: service request to unload a node
        :param client: service client to call the service
        """
        if not client.service_is_ready():
            self.__logger.info(
                "Could not unload node '{}'. Is '{}' down?".format(
                    self.__loaded_nodes[request.unique_id], self.__final_target_container_name,
                    client.srv_name
                )
            )
            return

        # Asynchronously wait on service call so that we can periodically check for shutdown
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        self.__logger.debug(
            "Calling the '{}' service with request '{}'".format(
                client.srv_name, request
            )
        )

        response_future = client.call_async(request)
        response_future.add_done_callback(unblock)

        timeout = 10.0
        while not event.wait(1.0):
            timeout -= 1.0
            if timeout <= 0.0:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to timeout.".format(
                        client.srv_name),
                )
                response_future.cancel()
                return

        # Get response
        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        self.__logger.debug("Received response '{}'".format(response))

        node_name = self.__loaded_nodes[request.unique_id]
        if response.success:
            del self.__loaded_nodes[request.unique_id]
            self.__logger.info("Unloaded node '{}' in container '{}'".format(
                node_name, self.__final_target_container_name
            ))
        else:
            self.__logger.error(
                "Failed to unload node '{}' in container '{}': {}".format(
                    node_name, self.__final_target_container_name,
                    response.error_message
                )
            )

    def _unload_in_sequence(
        self,
        unload_node_requests: List[composition_interfaces.srv.UnloadNode.Request],
        context: LaunchContext,
        ros_adapter: ROSAdapter = None,
        client: rclpy.client.Client = None
    ) -> None:
        """
        Unload composable nodes sequentially.

        :param unload_node_requests: a list of LoadNode service requests to execute
        :param context: current launch context
        :param ros_adapter: the ROS adapter to use for unloading
        :param client: the service client to use for unloading
        """
        # If this is the first call, a ROS node has to be created.
        # We need to create it ourselves, since the global ROS adapter also uses OnShutdown
        # to terminate, which may happen before our handler is called.
        if ros_adapter is None:
            # Init an auxiliary ROS adapter solely for processing the unloading requests.
            ros_adapter = ROSAdapter(autostart=False)
            ros_adapter.start(custom_name='launch_ros_{}_unload_nodes_{}'.format(os.getpid(), id(self)))
            # Create the service client.
            client = ros_adapter.ros_node.create_client(
                composition_interfaces.srv.UnloadNode, '{}/_container/unload_node'.format(
                    self.__final_target_container_name
                )
            )

        # Process the first request and schedule the rest.
        next_unload_node_request = unload_node_requests[0]
        unload_node_requests = unload_node_requests[1:]
        self._unload_node(next_unload_node_request, client)
        if len(unload_node_requests) > 0:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._unload_in_sequence, unload_node_requests, context, ros_adapter, client
                )
            )
        else:
            # If this was the last request, stop the ROS node.
            ros_adapter.shutdown()

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeEntitiesType]:
        """
        Unload all nodes when the launch context is shutdown.
        This handler is only registered if `unload_on_shutdown` is True.
        """
        # Generate unload requests
        unload_node_requests = []
        for unique_id, node_name in self.__loaded_nodes.items():
            request = composition_interfaces.srv.UnloadNode.Request()
            request.unique_id = unique_id
            unload_node_requests.append(request)

        # Process the unload requests
        if unload_node_requests:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._unload_in_sequence, unload_node_requests, context
                )
            )

        # Cancel the dummy coroutine so that the launch process can exit
        # The exit will happen once the futures from _unload_in_sequence complete
        if self.__no_shutdown_future is not None:
            self.__no_shutdown_future.cancel()

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return the future used to track completion."""
        return self.__no_shutdown_future

    def execute(
        self,
        context: LaunchContext
    ) -> Optional[List[Action]]:
        """Execute the action."""
        # resolve target container node name

        if is_a_subclass(self.__target_container, ComposableNodeContainer):
            self.__final_target_container_name = self.__target_container.node_name
        elif isinstance(self.__target_container, SomeSubstitutionsType_types_tuple):
            subs = normalize_to_list_of_substitutions(self.__target_container)
            self.__final_target_container_name = perform_substitutions(
                context, subs)
        else:
            self.__logger.error(
                'target container is neither a ComposableNodeContainer nor a SubstitutionType')
            return

        # Create a client to load nodes in the target container.
        self.__rclpy_load_node_client = get_ros_node(context).create_client(
            composition_interfaces.srv.LoadNode, '{}/_container/load_node'.format(
                self.__final_target_container_name
            )
        )

        # Generate load requests before execute() exits to avoid race with context changing
        # due to scope change (e.g. if loading nodes from within a GroupAction).
        load_node_requests = []
        autostart_actions = []
        for node_description in self.__composable_node_descriptions:
            request = get_composable_node_load_request(node_description, context)
            # The request can be None if the node description's condition evaluates to False
            if request is not None:
                load_node_requests.append(request)

            # If autostart is enabled, transition to the 'active' state.
            if hasattr(node_description, 'node_autostart') and node_description.node_autostart:
                complete_node_name = request.node_namespace + request.node_name
                if not complete_node_name.startswith('/'):
                    complete_node_name = '/' + complete_node_name
                self.__logger.info(
                    'Autostart enabled for requested lifecycle node {}'.format(complete_node_name))
                node_description.init_lifecycle_event_manager(context)
                autostart_actions.append(
                    LifecycleTransition(
                        lifecycle_node_names=[complete_node_name],
                        transition_ids=[lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE]
                    ))
                autostart_actions.append(
                    LifecycleTransition(
                        lifecycle_node_names=[complete_node_name],
                        transition_ids=[lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE]
                    ),
                )

        if load_node_requests:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._load_in_sequence, load_node_requests, context
                )
            )

        load_actions = super().execute(context)
        if load_actions is not None and len(autostart_actions) != 0:
            return load_actions + autostart_actions
        if load_actions is not None:
            return load_actions
        if len(autostart_actions) != 0:
            return autostart_actions

        if self.__unload_on_shutdown:
            # Register a shutdown event handler to unload the nodes when the launch context is shutdown.
            context.register_event_handler(
                OnShutdown(on_shutdown=self.__on_shutdown)
            )

            # Start dummy coroutine so that the launch process does not exit
            async def dummy_coroutine():
                try:
                    while True:
                        await asyncio.sleep(1.0)
                except asyncio.CancelledError:
                    pass
            self.__no_shutdown_future = context.asyncio_loop.create_task(
                dummy_coroutine()
            )

def get_composable_node_load_request(
    composable_node_description: ComposableNode,
    context: LaunchContext
):
    """Get the request that will be sent to the composable node container."""
    if composable_node_description.condition() is not None:
        if not composable_node_description.condition().evaluate(context):
            # Return no request if the node description's condition evaluates to False
            return None

    request = composition_interfaces.srv.LoadNode.Request()
    request.package_name = perform_substitutions(
        context, composable_node_description.package
    )
    request.plugin_name = perform_substitutions(
        context, composable_node_description.node_plugin
    )
    if composable_node_description.node_name is not None:
        request.node_name = perform_substitutions(
            context, composable_node_description.node_name
        )
    expanded_ns = composable_node_description.node_namespace
    if expanded_ns is not None:
        expanded_ns = perform_substitutions(context, expanded_ns)
    base_ns = context.launch_configurations.get('ros_namespace', None)
    combined_ns = make_namespace_absolute(prefix_namespace(base_ns, expanded_ns))
    if combined_ns is not None:
        request.node_namespace = combined_ns
    # request.log_level = perform_substitutions(context, node_description.log_level)
    remappings = []
    global_remaps = context.launch_configurations.get('ros_remaps', None)
    if global_remaps:
        remappings.extend([f'{src}:={dst}' for src, dst in global_remaps])
    if composable_node_description.remappings:
        remappings.extend([
            f'{perform_substitutions(context, src)}:={perform_substitutions(context, dst)}'
            for src, dst in composable_node_description.remappings
        ])
    if remappings:
        request.remap_rules = remappings
    params_container = context.launch_configurations.get('global_params', None)
    parameters = []
    if params_container is not None:
        for param in params_container:
            if isinstance(param, tuple):
                subs = normalize_parameter_dict({param[0]: param[1]})
                parameters.append(subs)
            else:
                param_file_path = Path(param).resolve()
                assert param_file_path.is_file()
                subs = ParameterFile(param_file_path)
                parameters.append(subs)
    if composable_node_description.parameters is not None:
        parameters.extend(list(composable_node_description.parameters))
    if parameters:
        request.parameters = [
            param.to_parameter_msg() for param in to_parameters_list(
                context, request.node_name, combined_ns,
                evaluate_parameters(
                    context, parameters
                )
            )
        ]

    if composable_node_description.extra_arguments is not None:
        request.extra_arguments = [
            param.to_parameter_msg() for param in to_parameters_list(
                context, request.node_name, combined_ns,
                evaluate_parameters(
                    context, composable_node_description.extra_arguments
                )
            )
        ]
    return request
