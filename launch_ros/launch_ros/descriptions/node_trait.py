# Copyright 2021 Southwest Research Institute, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
# Part 252.227-7013 or 7014 (Feb 2014).
#
# This notice must appear in all copies of this file and its derivatives.

"""Module for a description of a NodeTrait."""

from typing import TYPE_CHECKING

from launch import Action
from launch import LaunchContext

if TYPE_CHECKING:
    from . import Node


class NodeTrait:
    """Describes a trait of a node."""

    def __init__(self) -> None:
        """
        Initialize a NodeTrait description.

        Note that this class provides no functionality itself, and is used
        as a base class for traits which provide actual functionality. As
        such, the base class itself should not be directly used by application
        code.
        """
        pass

    def prepare(self, node: 'Node', context: LaunchContext, action: Action):
        """Perform any actions necessary to prepare the node for execution."""
        pass
