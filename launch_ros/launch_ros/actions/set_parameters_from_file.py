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

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions

import yaml
import os.path

@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    # TODO -- add description, short example

    """

    def __init__(
        self,
        filename,
        **kwargs
    ) -> None:
        """Create a SetParameterFromFile action."""
        super().__init__(**kwargs)
        if type(filename) == str:
            input_file = filename
        else:
            input_file = None

        assert os.path.isfile(input_file)
        with open(input_file, 'r') as stream:
            self.__global_params = yaml.safe_load(stream)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context.launch_configurations['global_params_from_file'] = self.__global_params
