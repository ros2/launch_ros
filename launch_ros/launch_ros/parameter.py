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

"""Module for a description of a Parameter."""

import collections.abc

from typing import Any
from typing import Optional
from typing import Text
from typing import Tuple
from typing import TYPE_CHECKING

from launch import LaunchContext
from launch import SomeSubstitutionsType
from launch import SomeSubstitutionsType_types_tuple
from launch import Substitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

if TYPE_CHECKING:
    from .parameters_type import SomeParameterValue
    from .parameters_type import EvaluatedParameterValue  # noqa: F401, false positive.


class Parameter:
    """Describes a ROS Parameter."""

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        value: 'SomeParameterValue',
        value_type: Any = None
    ) -> None:
        """
        Construct a parameter description.

        :param name: Name of the parameter.
        :param value: Value of the parameter.
        :param value_type: Used when `value` is a substitution, to coerce the result.
            Can be one of:
                - A scalar type: `int`, `str`, `float`, `bool`.
                  `bool` are written like in `yaml`.
                  Both `1` and `1.` are valid floats.
                - An uniform list: `List[int]`, `List[str]`, `List[float]`, `List[bool]`.
                  Lists are written like in `yaml`.
                - `None`, which means that yaml rules will be used.
                  The result of the convertion must be one of the above types,
                  if not `ValueError` is raised.
            If value is not a substitution and this parameter is provided,
            it will be used to check `value` type.
        """
        # Here to avoid cyclic dependency.
        from .utilities.type_utils import AllowedTypes
        from .parameters_type import SomeParameterValue_types_tuple

        ensure_argument_type(name, SomeSubstitutionsType_types_tuple, 'name')
        ensure_argument_type(value, SomeParameterValue_types_tuple, 'value')
        assert value_type is None or value_type in AllowedTypes, (
            f"expected `value_type` to be one of '{AllowedTypes + (None,)}, got {value_type}'"
        )

        self.__name = normalize_to_list_of_substitutions(name)
        self.__value = value
        self.__value_type = value_type
        self.__evaluated_parameter_name: Optional[Text] = None
        self.__evaluated_parameter_value: Optional['EvaluatedParameterValue'] = None
        self.__evaluated_parameter_rule: Optional[Tuple[Text, 'EvaluatedParameterValue']] = None

    @property
    def name(self):
        """Getter for parameter name."""
        if self.__evaluated_parameter_name is not None:
            return self.__evaluated_parameter_name
        return self.__name

    @property
    def value(self):
        """Getter for parameter value."""
        if self.__evaluated_parameter_value is not None:
            return self.__evaluated_parameter_value
        return self.__value

    @property
    def value_type(self):
        """Getter for parameter value type."""
        return self.__value_type

    def __str__(self):
        return f'ParameterDescription(name={self.name}, value={self.value})'

    def evaluate(self, context: LaunchContext) -> Tuple[Text, 'EvaluatedParameterValue']:
        """Evaluate and return parameter rule."""
        # Here to avoid cyclic import
        from .utilities.type_utils import AllowedTypes
        from .utilities.type_utils import coerce_to_type
        from .utilities.type_utils import check_is_instance_of_valid_type
        from .utilities.type_utils import check_type
        from .utilities.type_utils import extract_type

        if self.__evaluated_parameter_rule is not None:
            return self.__evaluated_parameter_rule

        name = perform_substitutions(context, self.name)
        value = self.__value
        # TODO(ivanpauno): Maybe this logic to convert a list should be provided by type_utils too.

        def is_substitution(x):
            return (
                isinstance(x, Substitution) or
                (
                    isinstance(x, collections.abc.Iterable) and
                    len(x) > 0 and
                    all(isinstance(item, (Substitution, str)) for item in x) and
                    any(isinstance(item, Substitution) for item in x)
                )
            )

        def convert_item(item, data_type):
            if is_substitution(item):
                item = perform_substitutions(
                    context,
                    normalize_to_list_of_substitutions(item))
                item = coerce_to_type(item, data_type)
            elif data_type is not None and not check_type(item, data_type):
                raise ValueError(
                    f"Expected provided 'value' to be of type '{data_type}',"
                    f' got `{type(item)}`'
                )
            elif not check_is_instance_of_valid_type(item):
                raise ValueError(
                    f"Expected 'value' to be instance of one of the following types"
                    f"'{AllowedTypes}', got '{type(item)}'"
                )
            return item
        if isinstance(value, list) and not is_substitution(value):
            data_type, is_list = extract_type(self.__value_type)
            if not is_list and self.__value_type is not None:
                raise ValueError(
                    f"Cannot convert input '{value}' of type '{type(value)}' to"
                    f" '{self.__value_type}'"
                )
            value = [convert_item(x, data_type) for x in value]
        else:
            value = convert_item(value, self.__value_type)

        self.__evaluated_parameter_name = name
        self.__evaluated_parameter_value = value
        self.__evaluated_parameter_rule = (name, value)
        return (name, value)
