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

from ament_index_python.resources import get_resource

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import Substitution, PathSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from typing import Sequence, Tuple, Type, Dict, Any, List, Text


@expose_substitution("ament-index-resource")
class AmentIndexResource(PathSubstitution):
    """
    Substitution that looks up the path for the given resource type and name.

    The resource is located using ament_index_python.
    """

    def __init__(
        self,
        type: SomeSubstitutionsType,
        name: SomeSubstitutionsType,
    ) -> None:
        """Create an AmentIndexResource substitution."""
        super().__init__(self)

        self.__type = normalize_to_list_of_substitutions(type)
        self.__name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type["AmentIndexResource"], Dict[str, Any]]:
        """Parse an AmentIndexResource subtitution."""
        if len(data) != 2:
            raise TypeError("ament-index-resource expects 2 arguments")
        kwargs = {
            "type": data[0],
            "name": data[1],
        }
        return cls, kwargs

    @property
    def type(self) -> List[Substitution]:
        """Getter for type."""
        return self.__type

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        type_str = " + ".join(sub.describe() for sub in self.type)
        name_str = " + ".join(sub.describe() for sub in self.name)
        return f"AmentIndexResource({type_str}, {name_str})"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the resource in the ament index."""
        type = perform_substitutions(context, self.type)
        name = perform_substitutions(context, self.name)
        return get_resource(type, name)[1]
