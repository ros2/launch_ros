# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module with utility to transform evaluated parameters into parameter lists."""

import pathlib
from typing import List

from launch.launch_context import LaunchContext
import rclpy.parameter

import yaml

from .evaluate_parameters import evaluate_parameter_dict
from .normalize_parameters import normalize_parameter_dict

from ..parameters_type import EvaluatedParameters


def search_for_key(keys, dictionary):
    """Search for the nested "ros__parameters" key and return the keys above it."""
    for key, value in dictionary.items():
        if key == "ros__parameters":
            return (keys, value)
        elif type(value) is dict:
            keys.append(key)
            return search_for_key(keys, value)
        else:
            return (None, None)


def to_parameters_list(
    context: LaunchContext,
    node_name: str,
    namespace: str,
    evaluated_parameters: EvaluatedParameters
) -> List[rclpy.parameter.Parameter]:
    """
    Transform evaluated parameters into a list of rclpy.parameter.Parameter objects.

    :param context: to carry out substitutions during normalization of parameter files.
        See `normalize_parameters()` documentation for further reference.
    :param node_name: node name
    :param namespace: node namespace
    :param parameters: parameters as either paths to parameter files or name/value pairs.
        See `evaluate_parameters()` documentation for further reference.
    :returns: a list of parameters
    """
    parameters = []  # type: List[rclpy.parameter.Parameter]
    for params_set_or_path in evaluated_parameters:
        if isinstance(params_set_or_path, pathlib.Path):
            with open(str(params_set_or_path), 'r') as f:
                node_name.lstrip('/')
                if namespace:
                    namespace.lstrip('/')
                    node_name = f"{namespace}/{node_name}"
                param_dict = yaml.safe_load(f)

                # Get all keys that come before "ros__parameters"
                keys = []
                inner_dict = {}
                (keys, inner_dict) = search_for_key(keys, param_dict)

                # If we found any, combine them to form the full name
                if keys:
                    keys = [key.lstrip('/') for key in keys]
                    yaml_name = '/'.join(keys)
                    if (yaml_name == node_name or yaml_name == "**"):
                        param_dict = inner_dict

                params_set = evaluate_parameter_dict(
                    context, normalize_parameter_dict(param_dict)
                )
        else:
            params_set = params_set_or_path
        if not isinstance(params_set, dict):
            raise RuntimeError('invalid evaluated parameters {}'.format(repr(params_set)))
        for name, value in params_set.items():
            type_ = next((
                type_ for type_ in rclpy.parameter.Parameter.Type if type_.check(value)
            ), None)  # type: rclpy.parameter.Parameter.Type
            if type_ is None:
                raise RuntimeError('invalid parameter value {}'.format(repr(value)))
            parameters.append(rclpy.parameter.Parameter(name, type_, value))
    return parameters
