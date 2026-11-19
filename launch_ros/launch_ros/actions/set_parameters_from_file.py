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

"""Module for the `SetParametersFromFile` action."""

from typing import List
from typing import Union

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution


@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    Action that sets parameters for all nodes in scope based on a given yaml file.

    By default, the parameters in the file are passed to each node in scope
    before the parameters defined on the node itself, so node parameters
    take precedence over the ones in the file.
    If `override` is true, the file is instead passed after the parameters
    defined on the node itself, so the parameters in the file take
    precedence over the node parameters.
    This makes it possible for a top-level launch file to override
    parameters that an included launch file defined on its nodes.
    When several files are registered with `override` set to true, they
    are applied in registration order, i.e. later files take precedence
    over earlier ones.

    For example:

    .. code-block:: python

        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetParametersFromFile('path/to/file.yaml'),
                    ...,
                    Node(...),  # the params will be passed to this node
                    ...,
                ]
            ),
            Node(...),  # here it won't be passed, as it's not in the same scope
            ...
        ])

    .. code-block:: xml

        <launch>
            <group>
                <set_parameters_from_file filename='/path/to/file.yaml'/>
                <node .../>  <!-- Node in scope, params will be passed -->
            </group>
            <node .../>  <!-- Node not in scope, params won't be passed -->
        </launch>
    """

    def __init__(
        self,
        filename: SomeSubstitutionsType,
        override: Union[bool, SomeSubstitutionsType] = False,
        **kwargs
    ) -> None:
        """Create a SetParameterFromFile action."""
        super().__init__(**kwargs)
        self._input_file = normalize_to_list_of_substitutions(filename)
        self.__override = normalize_typed_substitution(override, bool)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        override = entity.get_attr(
            'override', data_type=bool, optional=True, can_be_str=True)
        if override is not None:
            kwargs['override'] = parser.parse_if_substitutions(override)
        return cls, kwargs

    @property
    def override(self) -> List[Substitution]:
        """Getter for override."""
        return self.__override

    def execute(self, context: LaunchContext):
        """Execute the action."""
        filename = perform_substitutions(context, self._input_file)
        if perform_typed_substitution(context, self.__override, bool):
            # Register the file separately from the other global parameters,
            # so that nodes apply it after their own parameters, letting the
            # file take precedence over them.
            global_param_file_list = context.launch_configurations.get(
                'global_params_override_files', [])
            global_param_file_list.append(filename)
            context.launch_configurations['global_params_override_files'] = \
                global_param_file_list
        else:
            global_param_list = context.launch_configurations.get('global_params', [])
            global_param_list.append(filename)
            context.launch_configurations['global_params'] = global_param_list
