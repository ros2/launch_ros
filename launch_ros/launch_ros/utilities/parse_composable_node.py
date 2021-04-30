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
    """Parse composable_node"""

    attr_namespace = entity.get_attr('namespace', optional=True)
    if attr_namespace is not None:
        namespace = parser.parse_substitution(attr_namespace)
    else:
        namespace = None

    attr_parameters = entity.get_attr('param', data_type=List[Entity], optional=True)
    if attr_parameters is not None:
        parameters = Node.parse_nested_parameters(attr_parameters, parser)
    else:
        parameters = None

    attr_remappings = entity.get_attr('remap', data_type=List[Entity], optional=True)
    if attr_remappings is not None:
        remappings = [
            (
                parser.parse_substitution(remap.get_attr('from')),
                parser.parse_substitution(remap.get_attr('to'))
            ) for remap in attr_remappings
        ]

        for remap in attr_remappings:
            remap.assert_entity_completely_parsed()
    else:
        remappings = None

    attr_extra_arguments = entity.get_attr('extra_arg', data_type=List[Entity], optional=True)
    if attr_extra_arguments is not None:
        extra_arguments = Node.parse_nested_parameters(attr_extra_arguments, parser)
    else:
        extra_arguments = None

    return ComposableNode(
        package=parser.parse_substitution(entity.get_attr('pkg')),
        plugin=parser.parse_substitution(entity.get_attr('plugin')),
        name=parser.parse_substitution(entity.get_attr('name')),
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
        extra_arguments=extra_arguments,
    )
