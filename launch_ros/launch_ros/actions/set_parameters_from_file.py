# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module for the `SetParametersFromFile` action."""

import os.path

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType

import yaml


@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    Action that sets parameters for all nodes in scope based on a given yaml file.

    e.g.
    ```python3
        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetParametersFromFile('path/to/file.yaml'),
                    ...,
                    Node(...),  // the params will be passed to this node
                    ...,
                ]
            ),
            Node(...),  // here it won't be passed, as it's not in the same scope
            ...
        ])
    ```
    ```xml
    <launch>
        <group>
            <set_parameters_from_file filename='/path/to/file.yaml'/>
            node<.../>    // Node in scope, params will be passed
        </group>
        <node .../>  // Node not in scope, params won't be passed
    </launch>

    ```
    """

    def __init__(
        self,
        filename: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetParameterFromFile action."""
        super().__init__(**kwargs)
        input_file = self.extract_raw_text(filename)

        assert os.path.isfile(input_file)
        with open(input_file, 'r') as stream:
            self.__global_params = yaml.safe_load(stream)

    def extract_raw_text(self, substitution_object: SomeSubstitutionsType):
        if type(substitution_object) == list:
            return list(substitution_object[0].__dict__.values())[0]
        else:
            return substitution_object

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context.launch_configurations['global_params_from_file'] = self.__global_params
