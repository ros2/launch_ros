# Copyright 2025 Open Navigation LLC
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

"""Module for a description of a ComposableLifecycleNode."""

from typing import List
from typing import Optional

import launch
from launch.condition import Condition
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
# from launch.utilities import ensure_argument_type
from launch.utilities import perform_substitutions
from launch_ros.parameters_type import Parameters
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import RemapRules
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.utilities import LifecycleEventManager

from .composable_node import ComposableNode


class ComposableLifecycleNode(ComposableNode):
    """Describes a lifecycle node that can be loaded into a container with other nodes."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        plugin: SomeSubstitutionsType,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        autostart: Optional[bool] = False,
        remappings: Optional[SomeRemapRules] = None,
        extra_arguments: Optional[SomeParameters] = None,
        condition: Optional[Condition] = None,
    ) -> None:
        """
        Initialize a ComposableNode description.

        :param package: name of the ROS package the node plugin lives in
        :param plugin: name of the plugin to be loaded
        :param name: name to give to the ROS node
        :param namespace: namespace to give to the ROS node
        :param parameters: list of either paths to yaml files or dictionaries of parameters
        :param autostart: Whether to autostart lifecycle node in the activated state
        :param remappings: list of from/to pairs for remapping names
        :param extra_arguments: container specific arguments to be passed to the loaded node
        :param condition: action will be executed if the condition evaluates to true
        """
        super().__init__(
            package=package,
            plugin=plugin,
            name=name,
            namespace=namespace,
            parameters=parameters,
            remappings=remappings,
            extra_arguments=extra_arguments,
            condition=condition)

        self.__autostart = autostart
        self.__lifecycle_event_manager = None
        self.__node_name = self._ComposableNode__node_name

    def init_lifecycle_event_manager(self, node, context: launch.LaunchContext) -> None:
        # LifecycleEventManager needs a pre-substitution node name
        self.__node_name = perform_substitutions(context, node.node_name)
        self.__lifecycle_event_manager = LifecycleEventManager(node)
        self.__lifecycle_event_manager.setup_lifecycle_manager(context)

    @property
    def package(self) -> List[Substitution]:
        """Get node package name as a sequence of substitutions to be performed."""
        return self._ComposableNode__package

    @property
    def node_plugin(self) -> List[Substitution]:
        """Get node plugin name as a sequence of substitutions to be performed."""
        return self._ComposableNode__node_plugin

    @property
    def node_name(self) -> Optional[List[Substitution]]:
        """Get node name as a sequence of substitutions to be performed."""
        return self.__node_name

    @property
    def node_namespace(self) -> Optional[List[Substitution]]:
        """Get node namespace as a sequence of substitutions to be performed."""
        return self._ComposableNode__node_namespace

    @property
    def node_autostart(self):
        """Getter for autostart."""
        return self.__autostart

    @property
    def parameters(self) -> Optional[Parameters]:
        """Get node parameter YAML files or dicts with substitutions to be performed."""
        return self._ComposableNode__parameters

    @property
    def remappings(self) -> Optional[RemapRules]:
        """Get node remapping rules as (from, to) tuples with substitutions to be performed."""
        return self._ComposableNode__remappings

    @property
    def extra_arguments(self) -> Optional[Parameters]:
        """Get container extra arguments YAML files or dicts with substitutions to be performed."""
        return self._ComposableNode__extra_arguments
