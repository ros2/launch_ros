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

"""Module for a description of a ComposableNode."""

from typing import List
from typing import Optional
import warnings

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
# from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.parameters_type import Parameters
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import RemapRules
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.utilities import normalize_parameters
from launch_ros.utilities import normalize_remap_rules


class ComposableNode:
    """Describes a ROS node that can be loaded into a container with other nodes."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        plugin: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        node_plugin: Optional[SomeSubstitutionsType] = None,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        extra_arguments: Optional[SomeParameters] = None,
    ) -> None:
        """
        Initialize a ComposableNode description.

        .. deprecated:: Foxy
           Parameters `node_plugin`, `node_name`, and `node_namespace` are deprecated.
           Use `plugin`, `name`, and `namespace` instead.

        :param package: name of the ROS package the node plugin lives in
        :param plugin: name of the plugin to be loaded
        :param name: name to give to the ROS node
        :param namespace: namespace to give to the ROS node
        :param node_plugin: (DEPRECATED) name of the plugin to be loaded
        :param node_name: (DEPRECATED) name the node should have
        :param node_namespace: (DEPRECATED) namespace the node should create topics/services/etc in
        :param parameters: list of either paths to yaml files or dictionaries of parameters
        :param remappings: list of from/to pairs for remapping names
        :param extra_arguments: container specific arguments to be passed to the loaded node
        """
        if node_plugin is not None:
            warnings.warn("The parameter 'node_plugin' is deprecated, use 'plugin' instead")
            if plugin is not None:
                raise RuntimeError(
                    "Passing both 'node_plugin' and 'plugin' parameters. Only use 'plugin'."
                )
            plugin = node_plugin
        if plugin is None:
            raise RuntimeError("The 'plugin' parameter is required")
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

        self.__package = normalize_to_list_of_substitutions(package)
        self.__node_plugin = normalize_to_list_of_substitutions(plugin)

        self.__node_name = None  # type: Optional[List[Substitution]]
        if name is not None:
            self.__node_name = normalize_to_list_of_substitutions(name)

        self.__node_namespace = None  # type: Optional[List[Substitution]]
        if namespace is not None:
            self.__node_namespace = normalize_to_list_of_substitutions(namespace)

        self.__parameters = None  # type: Optional[Parameters]
        if parameters is not None:
            self.__parameters = normalize_parameters(parameters)

        self.__remappings = None  # type: Optional[RemapRules]
        if remappings:
            self.__remappings = normalize_remap_rules(remappings)

        self.__extra_arguments = None  # type: Optional[Parameters]
        if extra_arguments:
            self.__extra_arguments = normalize_parameters(extra_arguments)

    @property
    def package(self) -> List[Substitution]:
        """Get node package name as a sequence of substitutions to be performed."""
        return self.__package

    @property
    def node_plugin(self) -> List[Substitution]:
        """Get node plugin name as a sequence of substitutions to be performed."""
        return self.__node_plugin

    @property
    def node_name(self) -> Optional[List[Substitution]]:
        """Get node name as a sequence of substitutions to be performed."""
        return self.__node_name

    @property
    def node_namespace(self) -> Optional[List[Substitution]]:
        """Get node namespace as a sequence of substitutions to be performed."""
        return self.__node_namespace

    @property
    def parameters(self) -> Optional[Parameters]:
        """Get node parameter YAML files or dicts with substitutions to be performed."""
        return self.__parameters

    @property
    def remappings(self) -> Optional[RemapRules]:
        """Get node remapping rules as (from, to) tuples with substitutions to be performed."""
        return self.__remappings

    @property
    def extra_arguments(self) -> Optional[Parameters]:
        """Get container extra arguments YAML files or dicts with substitutions to be performed."""
        return self.__extra_arguments
