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

"""Module for the ComposableNodeContainer action."""

from typing import List
from typing import Optional
import warnings

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType

from .node import Node

from ..descriptions import ComposableNode


class ComposableNodeContainer(Node):
    """Action that executes a container ROS node for composable ROS nodes."""

    def __init__(
        self,
        *,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        composable_node_descriptions: Optional[List[ComposableNode]] = None,
        **kwargs
    ) -> None:
        """
        Construct a ComposableNodeContainer action.

        Most arguments are forwarded to :class:`launch_ros.actions.Node`, so see the documentation
        of that class for further details.

        .. deprecated:: Foxy
           Parameters `node_name` and `node_namespace` are deprecated.
           Use `name` and `namespace` instead.

        :param: name the name of the node, mandatory for full container node name resolution
        :param: namespace the ROS namespace for this Node, mandatory for full container node
             name resolution
        :param: node_name (DEPRECATED) the name of the node, mandatory for full container node
            name resolution
        :param: node_namespace (DEPRECATED) the ros namespace for this Node, mandatory for full
            container node name resolution
        :param composable_node_descriptions: optional descriptions of composable nodes to be loaded
        """
        if node_name is not None:
            warnings.warn("The parameter 'node_name' is deprecated, use 'name' instead")
            if name is not None:
                raise RuntimeError(
                    "Passing both 'node_name' and 'name' parameters. Only use 'name'."
                )
            name = node_name
        if node_namespace is not None:
            warnings.warn("The parameter 'node_namespace' is deprecated, use 'namespace' instead")
            if namespace is not None:
                raise RuntimeError(
                    "Passing both 'node_namespace' and 'namespace' parameters. "
                    "Only use 'namespace'."
                )
            namespace = node_namespace

        if name is None:
            raise RuntimeError("'name' is a required argument")

        if namespace is None:
            raise RuntimeError("'namespace' is a required argument")

        if namespace == '':
            namespace = '/'
        super().__init__(name=name, namespace=namespace, **kwargs)
        self.__composable_node_descriptions = composable_node_descriptions

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
        composable nodes load action if it applies.
        """
        load_actions = None  # type: Optional[List[Action]]
        if (
            self.__composable_node_descriptions is not None and
            len(self.__composable_node_descriptions) > 0
        ):
            from .load_composable_nodes import LoadComposableNodes
            # Perform load action once the container has started.
            load_actions = [
                LoadComposableNodes(
                    composable_node_descriptions=self.__composable_node_descriptions,
                    target_container=self
                )
            ]
        container_actions = super().execute(context)  # type: Optional[List[Action]]
        if container_actions is not None and load_actions is not None:
            return container_actions + load_actions
        if container_actions is not None:
            return container_actions
        if load_actions is not None:
            return load_actions
        return None
