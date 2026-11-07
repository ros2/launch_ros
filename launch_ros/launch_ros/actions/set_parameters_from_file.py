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

from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Optional

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.frontend.parse_substitution import parse_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution

import yaml


@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    Action that sets parameters for all nodes in scope based on a given yaml file.

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

    The `allow_substs` parameter can be used to enable substitutions in the
    parameter file. When enabled, substitutions like $(env VAR_NAME) will be
    processed before the file is used.
    """

    def __init__(
        self,
        filename: SomeSubstitutionsType,
        *,
        allow_substs: [bool, SomeSubstitutionsType] = False,
        **kwargs
    ) -> None:
        """
        Create a SetParameterFromFile action.

        :param filename: Path to a parameter file.
        :param allow_substs: Allow substitutions in the parameter file.
        """
        super().__init__(**kwargs)
        self._input_file = normalize_to_list_of_substitutions(filename)
        self._allow_substs = normalize_typed_substitution(allow_substs, data_type=bool)
        self._created_tmp_file = False
        self._tmp_file_path: Optional[Path] = None

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        allow_substs = entity.get_attr("allow_substs", optional=True)
        if allow_substs is not None:
            allow_substs = parser.parse_substitution(allow_substs)
            kwargs["allow_substs"] = allow_substs
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        filename = perform_substitutions(context, self._input_file)
        allow_substs = perform_typed_substitution(
            context, self._allow_substs, data_type=bool
        )

        param_file_path: Path = Path(filename)
        if allow_substs:
            with open(param_file_path, "r") as f, NamedTemporaryFile(
                mode="w", prefix="launch_params_", delete=False
            ) as h:
                parsed = perform_substitutions(context, parse_substitution(f.read()))
                try:
                    yaml.safe_load(parsed)
                except Exception:
                    raise SubstitutionFailure(
                        "The substituted parameter file is not a valid yaml file"
                    )
                h.write(parsed)
                param_file_path = Path(h.name)
                self._created_tmp_file = True
                self._tmp_file_path = param_file_path

        global_param_list = context.launch_configurations.get('global_params', [])
        global_param_list.append(str(param_file_path))
        context.launch_configurations['global_params'] = global_param_list
