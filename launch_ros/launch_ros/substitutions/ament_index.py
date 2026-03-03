# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Module for the AmentIndex substitution."""

from typing import Any, Dict, List, Sequence, Text, Tuple, Type

from ament_index_python.resources import get_resource

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import PathSubstitution, SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@expose_substitution('ament-index-resource')
class AmentIndexResource(PathSubstitution):
    """
    Substitution that looks up the path for the given resource type and name.

    The resource is located using ament_index_python.
    """

    def __init__(
        self,
        resource_type: SomeSubstitutionsType,
        resource_name: SomeSubstitutionsType,
    ) -> None:
        """Create an AmentIndexResource substitution."""
        super().__init__(self)

        self.__type = normalize_to_list_of_substitutions(resource_type)
        self.__name = normalize_to_list_of_substitutions(resource_name)

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type['AmentIndexResource'], Dict[str, Any]]:
        """Parse an AmentIndexResource subtitution."""
        if len(data) != 2:
            raise TypeError('ament-index-resource expects 2 arguments')
        kwargs = {
            'resource_type': data[0],
            'resource_name': data[1],
        }
        return cls, kwargs

    @property
    def resource_type(self) -> List[Substitution]:
        """Getter for resource_type."""
        return self.__type

    @property
    def resource_name(self) -> List[Substitution]:
        """Getter for resource_name."""
        return self.__name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        type_str = ' + '.join(sub.describe() for sub in self.resource_type)
        name_str = ' + '.join(sub.describe() for sub in self.resource_name)
        return f'AmentIndexResource({type_str}, {name_str})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the resource in the ament index."""
        resource_type = perform_substitutions(context, self.resource_type)
        resource_name = perform_substitutions(context, self.resource_name)
        try:
            return get_resource(resource_type, resource_name)[1]
        except Exception as e:
            raise SubstitutionFailure(e)
