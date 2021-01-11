# Copyright 2021 Southwest Research Institute, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
# Part 252.227-7013 or 7014 (Feb 2014).
#
# This notice must appear in all copies of this file and its derivatives.

"""Module for a description of a Node."""

import os
import pathlib

from tempfile import NamedTemporaryFile

from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union

from launch import LaunchContext
from launch import SomeSubstitutionsType
from launch.descriptions import Executable
import launch.logging
from launch.substitutions import LocalSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_ros.utilities import add_node_name
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import get_node_name_count
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import normalize_parameters
from launch_ros.utilities import normalize_remap_rules
from launch_ros.utilities import prefix_namespace

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import yaml

from .node_trait import NodeTrait
from ..parameter_descriptions import Parameter
from ..parameters_type import SomeParameters
from ..remap_rule_type import SomeRemapRules


class Node:
    """Describes a ROS node."""

    UNSPECIFIED_NODE_NAME = '<node_name_unspecified>'
    UNSPECIFIED_NODE_NAMESPACE = '<node_namespace_unspecified>'

    def __init__(
        self, *,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        traits: Optional[Iterable[NodeTrait]] = None,
        **kwargs
    ) -> None:
        """
        Construct an Node description.

        Many arguments are passed eventually to
        :class:`launch.actions.ExecuteProcess`, so see the documentation of
        that class for additional details.

        If the node name is not given (or is None) then no name is passed to
        the node on creation and instead the default name specified within the
        code of the node is used instead.

        The namespace can either be absolute (i.e. starts with /) or
        relative.
        If absolute, then nothing else is considered and this is passed
        directly to the node to set the namespace.
        If relative, the namespace in the 'ros_namespace' LaunchConfiguration
        will be prepended to the given relative node namespace.
        If no namespace is given, then the default namespace `/` is
        assumed.

        The parameters are passed as a list, with each element either a yaml
        file that contains parameter rules (string or pathlib.Path to the full
        path of the file), or a dictionary that specifies parameter rules.
        Keys of the dictionary can be strings or an iterable of Substitutions
        that will be expanded to a string.
        Values in the dictionary can be strings, integers, floats, or tuples
        of Substitutions that will be expanded to a string.
        Additionally, values in the dictionary can be lists of the
        aforementioned types, or another dictionary with the same properties.
        A yaml file with the resulting parameters from the dictionary will be
        written to a temporary file, the path to which will be passed to the
        node.
        Multiple dictionaries/files can be passed: each file path will be
        passed in in order to the node (where the last definition of a
        parameter takes effect).

        :param: node_name the name of the node
        :param: node_namespace the ROS namespace for this Node
        :param: parameters list of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: arguments list of extra arguments for the node
        :param: traits list of special traits of the node
        """
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            # All elements in the list are paths to files with parameters (or substitutions that
            # evaluate to paths), or dictionaries of parameters (fields can be substitutions).
            normalized_params = normalize_parameters(parameters)

        self.__node_name = node_name
        self.__node_namespace = node_namespace
        self.__parameters = [] if parameters is None else normalized_params
        self.__remappings = [] if remappings is None else list(normalize_remap_rules(remappings))
        self.__arguments = arguments
        self.__traits = traits

        self.__expanded_node_name = self.UNSPECIFIED_NODE_NAME
        self.__expanded_node_namespace = self.UNSPECIFIED_NODE_NAMESPACE
        self.__expanded_parameter_arguments = None  # type: Optional[List[Tuple[Text, bool]]]
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_remappings = None  # type: Optional[List[Tuple[Text, Text]]]

        self.__substitutions_performed = False

        self.__logger = launch.logging.get_logger(__name__)

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    @property
    def node_namespace(self):
        """Getter for node_namespace."""
        return self.__node_namespace

    @property
    def parameters(self):
        """Getter for parameters."""
        return self.__parameters

    @property
    def remappings(self):
        """Getter for remappings."""
        return self.__remappings

    @property
    def arguments(self):
        """Getter for arguments."""
        return self.__arguments

    @property
    def traits(self):
        """Getter for traits."""
        return self.__traits

    @property
    def expanded_node_name(self):
        """Getter for expanded_node_name."""
        return self.__expanded_node_name

    @property
    def expanded_node_namespace(self):
        """Getter for expanded_node_namespace."""
        return self.__expanded_node_namespace

    @property
    def expanded_parameter_arguments(self):
        """Getter for expanded_parameter_arguments."""
        return self.__expanded_parameter_arguments

    @property
    def expanded_remappings(self):
        """Getter for expanded_remappings."""
        return self.__expanded_remappings

    def is_node_name_fully_specified(self):
        keywords = (self.UNSPECIFIED_NODE_NAME, self.UNSPECIFIED_NODE_NAMESPACE)
        return all(x not in self.node_name for x in keywords)

    def _create_params_file_from_dict(self, params):
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            param_dict = {
                self.node_name if self.is_node_name_fully_specified() else '/**':
                {'ros__parameters': params}
            }
            yaml.dump(param_dict, h, default_flow_style=False)
            return param_file_path

    def _get_parameter_rule(self, param: 'Parameter', context: LaunchContext):
        name, value = param.evaluate(context)
        return f'{name}:={yaml.dump(value)}'

    def prepare(self, context: LaunchContext, executable: Executable) -> None:
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            cmd_ext = ['--ros-args']  # Prepend ros specific arguments with --ros-args flag
            executable.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_ext])
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__expanded_node_name)
            self.__expanded_node_name.lstrip('/')
            expanded_node_namespace: Optional[Text] = None
            if self.__node_namespace is not None:
                expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            base_ns = context.launch_configurations.get('ros_namespace', None)
            expanded_node_namespace = make_namespace_absolute(
                prefix_namespace(base_ns, expanded_node_namespace))
            if expanded_node_namespace is not None:
                self.__expanded_node_namespace = expanded_node_namespace
                cmd_ext = ['-r', LocalSubstitution("ros_specific_arguments['ns']")]
                executable.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_ext])
                validate_namespace(self.__expanded_node_namespace)
        except Exception:
            self.__logger.error(
                "Error while expanding or validating node name or namespace for '{}':"
                .format('name={}, namespace={}'.format(
                    self.__node_name,
                    self.__node_namespace,
                ))
            )
            raise
        self.__final_node_name = prefix_namespace(
            self.__expanded_node_namespace, self.__expanded_node_name)
        # expand global parameters first,
        # so they can be overriden with specific parameters of this Node
        global_params = context.launch_configurations.get('ros_params', None)
        if global_params is not None or self.__parameters is not None:
            self.__expanded_parameter_arguments = []
        if global_params is not None:
            param_file_path = self._create_params_file_from_dict(global_params)
            self.__expanded_parameter_arguments.append((param_file_path, True))
            cmd_ext = ['--params-file', f'{param_file_path}']
            executable.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_ext])
            assert os.path.isfile(param_file_path)
        # expand parameters too
        if self.__parameters is not None:
            evaluated_parameters = evaluate_parameters(context, self.__parameters)
            for params in evaluated_parameters:
                is_file = False
                if isinstance(params, dict):
                    param_argument = self._create_params_file_from_dict(params)
                    is_file = True
                    assert os.path.isfile(param_argument)
                elif isinstance(params, pathlib.Path):
                    param_argument = str(params)
                    is_file = True
                elif isinstance(params, Parameter):
                    param_argument = self._get_parameter_rule(params, context)
                else:
                    raise RuntimeError('invalid normalized parameters {}'.format(repr(params)))
                if is_file and not os.path.isfile(param_argument):
                    self.__logger.warning(
                        'Parameter file path is not a file: {}'.format(param_argument),
                    )
                    continue
                self.__expanded_parameter_arguments.append((param_argument, is_file))
                cmd_ext = ['--params-file' if is_file else '-p', f'{param_argument}']
                executable.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_ext])
        # expand remappings too
        global_remaps = context.launch_configurations.get('ros_remaps', None)
        if global_remaps or self.__remappings:
            self.__expanded_remappings = []
        if global_remaps:
            self.__expanded_remappings.extend(global_remaps)
        if self.__remappings:
            self.__expanded_remappings.extend([
                (perform_substitutions(context, src), perform_substitutions(context, dst))
                for src, dst in self.__remappings
            ])
        if self.__expanded_remappings:
            cmd_ext = []
            for src, dst in self.__expanded_remappings:
                cmd_ext.extend(['-r', f'{src}:={dst}'])
            executable.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_ext])
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
        if self.__node_name is not None:
            ros_specific_arguments['name'] = '__node:={}'.format(self.__expanded_node_name)
        if self.__expanded_node_namespace != '':
            ros_specific_arguments['ns'] = '__ns:={}'.format(self.__expanded_node_namespace)
        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})

        if self.is_node_name_fully_specified():
            add_node_name(context, self.node_name)
            node_name_count = get_node_name_count(context, self.node_name)
            if node_name_count > 1:
                execute_process_logger = launch.logging.get_logger(self.name)
                execute_process_logger.warning(
                    'there are now at least {} nodes with the name {} created within this '
                    'launch context'.format(node_name_count, self.node_name)
                )
