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
from launch.substitution import Substitution
from launch.utilities import perform_substitutions
from launch_ros.parameters_type import Parameters
from launch_ros.remap_rule_type import RemapRules
from launch_ros.utilities import LifecycleEventManager

from .composable_node import ComposableNode


class ComposableLifecycleNode(ComposableNode):
    """Describes a lifecycle node that can be loaded into a container with other nodes."""

    def __init__(
        self, *,
        autostart: bool = False,
        **kwargs
    ) -> None:
        """
        Initialize a ComposableLifecycleNode description.

        :param autostart: Whether to autostart lifecycle node in the activated state
        """
        super().__init__(**kwargs)

        self.__autostart = autostart
        self.__lifecycle_event_manager = None
        self.__node_name = super().node_name

    def init_lifecycle_event_manager(self, context: launch.LaunchContext) -> None:
        # LifecycleEventManager needs a pre-substitution node name
        self.__node_name = perform_substitutions(context, self.node_name)
        self.__lifecycle_event_manager = LifecycleEventManager(self)
        self.__lifecycle_event_manager.setup_lifecycle_manager(context)

    @property
    def package(self) -> List[Substitution]:
        """Get node package name as a sequence of substitutions to be performed."""
        return super().package

    @property
    def node_plugin(self) -> List[Substitution]:
        """Get node plugin name as a sequence of substitutions to be performed."""
        return super().node_plugin

    @property
    def node_name(self) -> Optional[List[Substitution]]:
        """Get node name as a sequence of substitutions to be performed."""
        return self.__node_name

    @property
    def node_namespace(self) -> Optional[List[Substitution]]:
        """Get node namespace as a sequence of substitutions to be performed."""
        return super().node_namespace

    @property
    def node_autostart(self):
        """Getter for autostart."""
        return self.__autostart

    @property
    def parameters(self) -> Optional[Parameters]:
        """Get node parameter YAML files or dicts with substitutions to be performed."""
        return super().parameters

    @property
    def remappings(self) -> Optional[RemapRules]:
        """Get node remapping rules as (from, to) tuples with substitutions to be performed."""
        return super().remappings

    @property
    def extra_arguments(self) -> Optional[Parameters]:
        """Get container extra arguments YAML files or dicts with substitutions to be performed."""
        return super().extra_arguments
