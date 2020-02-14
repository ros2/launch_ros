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

"""Module which implements get_typed_value function."""

from typing import Any
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

import yaml

__ScalarTypesTuple = (
    int, float, bool, str
)

AllowedTypes = __ScalarTypesTuple + tuple(List[x] for x in __ScalarTypesTuple)


def check_is_typing_list(data_type: Any) -> bool:
    """Check if `data_type` is based on a `typing.List`."""
    return hasattr(data_type, '__origin__') and \
        data_type.__origin__ in (list, List)  # On Linux/Mac is List, on Windows is list.


def check_valid_scalar_type(data_type: Any) -> bool:
    """Check if `data_type` is a valid scalar type."""
    return data_type in __ScalarTypesTuple


def extract_type(data_type: Any) -> Tuple[Any, bool]:
    """
    Extract type information from type object.

    :param data_type: It can be:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;

    :returns: a tuple (type_obj, is_list).
        is_list is `True` for the supported list types, if not is `False`.
        type_obj is the object representing that type in python. In the case of list
        is the type of the items.
        e.g.:
            `name = List[int]` -> `(int, True)`
            `name = bool` -> `(bool, False)`
            `name = List[Union[bool, str]]` -> `(Union[bool, str], True)`
    """
    is_list = False
    if check_is_typing_list(data_type):
        is_list = True
        data_type = data_type.__args__[0]
    if data_type is not None and check_valid_scalar_type(data_type) is False:
        raise ValueError('Unrecognized data type: {}'.format(data_type))
    return (data_type, is_list)


def check_type(value: Any, data_type: Any) -> bool:
    """
    Check if `value` is of `type`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;

    `types = None` works in the same way as:
        `Union[int, float, bool, list, str]`
    """
    def check_scalar_type(value, data_type):
        return isinstance(value, data_type)
    type_obj, is_list = extract_type(data_type)
    if not is_list:
        return check_scalar_type(value, type_obj)
    if not isinstance(value, list) or not value:
        return False
    return all(check_scalar_type(x, type_obj) for x in value)


def check_is_instance_of_valid_type(value):
    if isinstance(value, list):
        if not value:
            return True  # Accept empty lists.
        member_type = type(value[0])
        return (
            all(isinstance(x, member_type) for x in value[1:]) and
            member_type in __ScalarTypesTuple
        )
    return isinstance(value, __ScalarTypesTuple)


def coerce_to_type(
    value: Text,
    data_type: Any = None
) -> Union[
    List[int],
    List[str],
    List[float],
    List[bool],
    int, str, float, bool,
]:
    """
    Try to convert `value` to the type specified in `data_type`.

    If not raise `ValueError`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
        - `None`: try to use yaml convertion rules, and checks if the output is
          a scalar or an uniform list.

    The coercion order for scalars is always: `int`, `float`, `bool`, `str`.
    """
    # TODO(ivanpauno): We could probably implement our own parser, instead of restrict yaml
    # conversions in this way.
    def convert_as_yaml(value, error_msg):
        try:
            output = yaml.safe_load(value)
        except Exception as err:
            raise ValueError(f'{error_msg}: yaml.safe_load() failed\n{err}')

        if not check_is_instance_of_valid_type(output):
            raise ValueError(
                f'{error_msg}: output type is not allowed, got {type(output)}'
            )
        return output

    if data_type is None:
        # No type specified, return best conversion.
        return convert_as_yaml(value, f"Failed to convert '{value}' using yaml rules")

    type_obj, is_list = extract_type(data_type)

    if is_list:
        output = convert_as_yaml(
            value, f"Cannot convert value '{value}' to a list of '{type_obj}'")
        if not isinstance(output, list):
            raise ValueError(f"Cannot convert value '{value}' to a list of '{type_obj}'")
        if not output:
            return output  # return empty lists
        if not all(isinstance(x, type_obj) for x in output):
            raise ValueError(
                f"Cannot convert value '{value}' to a list of '{type_obj}', got {output}")
        return output

    if type_obj is str:
        return value
    if type_obj in (int, float):
        return type_obj(value)

    assert bool == type_obj, 'This error should not happen, please open an issue'
    output = convert_as_yaml(value, f"Failed to convert '{value}' to '{type_obj}'")
    if isinstance(output, type_obj):
        return output
    raise ValueError(f"Cannot convert value '{value}' to '{type_obj}'")
