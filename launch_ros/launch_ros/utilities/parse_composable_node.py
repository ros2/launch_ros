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

"""Module containing parser for composable_node tag."""

from typing import List

from launch.frontend import Entity
from launch.frontend import Parser
from launch_ros.actions.node import Node
from launch_ros.descriptions import ComposableNode


def parse_composable_node(parser: Parser, entity: Entity):
    """Parse composable_node."""
    kwargs = {}

    kwargs['package'] = parser.parse_substitution(entity.get_attr('pkg'))
    kwargs['plugin'] = parser.parse_substitution(entity.get_attr('plugin'))
    kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))

    namespace = entity.get_attr('namespace', optional=True)
    if namespace is not None:
        kwargs['namespace'] = parser.parse_substitution(namespace)

    parameters = entity.get_attr('param', data_type=List[Entity], optional=True)
    if parameters is not None:
        kwargs['parameters'] = Node.parse_nested_parameters(parameters, parser)

    remappings = entity.get_attr('remap', data_type=List[Entity], optional=True)
    if remappings is not None:
        kwargs['remappings'] = [
            (
                parser.parse_substitution(remap.get_attr('from')),
                parser.parse_substitution(remap.get_attr('to'))
            ) for remap in remappings
        ]

        for remap in remappings:
            remap.assert_entity_completely_parsed()

    extra_arguments = entity.get_attr('extra_arg', data_type=List[Entity], optional=True)
    if extra_arguments is not None:
        kwargs['extra_arguments'] = Node.parse_nested_parameters(extra_arguments, parser)

    return (ComposableNode, kwargs)
