# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Module for the Parameter substitution."""

import collections.abc
from typing import Any
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Text
from typing import Union

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


@expose_substitution('param')
class Parameter(Substitution):
    """
    Substitution that tries to get a parameter that was set by SetParameter.

    :raise: SubstitutionFailure when param is not found
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        *,
        default: Optional[Union[Any, Iterable[Any]]] = None
    ) -> None:
        """Create a Parameter substitution."""
        super().__init__()
        self.__name = normalize_to_list_of_substitutions(name)
        if default is None:
            self.__default = default
        else:
            # convert any items in default that are not a Substitution or str to a str
            str_normalized_default = []  # type: List[Union[Text, Substitution]]
            definitely_iterable_default = ((),)  # type: Iterable[Any]
            if isinstance(default, collections.abc.Iterable):
                definitely_iterable_default = default
            else:
                definitely_iterable_default = (default,)
            for item in definitely_iterable_default:
                if isinstance(item, (str, Substitution)):
                    str_normalized_default.append(item)
                else:
                    str_normalized_default.append(str(item))
            # use normalize_to_list_of_substitutions to convert str to TextSubstitution's too
            self.__default = \
                normalize_to_list_of_substitutions(str_normalized_default)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse a Parameter substitution."""
        if len(data) < 1 or len(data) > 2:
            raise TypeError('param substitution expects 1 or 2 arguments')
        kwargs = {}
        kwargs['name'] = data[0]
        if len(data) == 2:
            kwargs['default'] = data[1]
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        name_str = ' + '.join([sub.describe() for sub in self.name])
        return '{}(name={})'.format(self.__class__.__name__, name_str)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        name = perform_substitutions(context, self.name)
        params_container = context.launch_configurations.get('global_params', [])
        for param in params_container:
            if isinstance(param, tuple):
                if param[0] == name:
                    return str(param[1])

        if self.__default is None:
            raise SubstitutionFailure("parameter '{}' not found".format(name))
        else:
            return perform_substitutions(context, self.__default)
