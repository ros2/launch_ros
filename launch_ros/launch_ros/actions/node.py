# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the Node action."""

import os
import pathlib
from tempfile import NamedTemporaryFile
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401

from launch.action import Action
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext

import launch.logging

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LocalSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_frontend import Entity, expose_action

from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import normalize_parameters
from launch_ros.utilities import normalize_remap_rules

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import yaml


@expose_action('node')
class Node(ExecuteProcess):
    """Action that executes a ROS node."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        node_executable: SomeSubstitutionsType,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """
        Construct an Node action.

        Many arguments are passed eventually to
        :class:`launch.actions.ExecuteProcess`, so see the documentation of
        that class for additional details.
        However, the `cmd` is not meant to be used, instead use the
        `node_executable` and `arguments` keyword arguments to this function.

        This action, once executed, delegates most work to the
        :class:`launch.actions.ExecuteProcess`, but it also converts some ROS
        specific arguments into generic command line arguments.

        The launch_ros.substitutions.ExecutableInPackage substitution is used
        to find the executable at runtime, so this Action also raise the
        exceptions that substituion can raise when the package or executable
        are not found.

        If the node_name is not given (or is None) then no name is passed to
        the node on creation and instead the default name specified within the
        code of the node is used instead.

        The node_namespace can either be absolute (i.e. starts with /) or
        relative.
        If absolute, then nothing else is considered and this is passed
        directly to the node to set the namespace.
        If relative, the namespace in the 'ros_namespace' LaunchConfiguration
        will be prepended to the given relative node namespace.
        If no node_namespace is given, then the default namespace `/` is
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

        :param: package the package in which the node executable can be found
        :param: node_executable the name of the executable to find
        :param: node_name the name of the node
        :param: node_namespace the ros namespace for this Node
        :param: parameters list of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: arguments list of extra arguments for the node
        """
        cmd = [ExecutableInPackage(package=package, executable=node_executable)]
        cmd += [] if arguments is None else arguments
        # Reserve space for ros specific arguments.
        # The substitutions will get expanded when the action is executed.
        ros_args_index = 0
        if node_name is not None:
            cmd += [LocalSubstitution(
                'ros_specific_arguments[{}]'.format(ros_args_index), description='node name')]
            ros_args_index += 1
        if node_namespace is not None:
            cmd += [LocalSubstitution(
                'ros_specific_arguments[{}]'.format(ros_args_index), description='node namespace')]
            ros_args_index += 1
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            # All elements in the list are paths to files with parameters (or substitutions that
            # evaluate to paths), or dictionaries of parameters (fields can be substitutions).
            i = 0
            for param in parameters:
                i += 1
                cmd += [LocalSubstitution(
                    'ros_specific_arguments[{}]'.format(ros_args_index),
                    description='parameter {}'.format(i))]
                ros_args_index += 1
            normalized_params = normalize_parameters(parameters)
        if remappings is not None:
            i = 0
            for remapping in normalize_remap_rules(remappings):
                k, v = remapping
                i += 1
                cmd += [LocalSubstitution(
                    'ros_specific_arguments[{}]'.format(ros_args_index),
                    description='remapping {}'.format(i))]
                ros_args_index += 1
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = node_executable
        self.__node_name = node_name
        self.__node_namespace = node_namespace
        self.__parameters = [] if parameters is None else normalized_params
        self.__remappings = [] if remappings is None else remappings
        self.__arguments = arguments

        self.__expanded_node_name = '<node_name_unspecified>'
        self.__expanded_node_namespace = '/'
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_parameter_files = None  # type: Optional[List[Text]]
        self.__expanded_remappings = None  # type: Optional[List[Tuple[Text, Text]]]

        self.__substitutions_performed = False

        self.__logger = launch.logging.get_logger(__name__)

    @staticmethod
    def parse_nested_parameters(params):
        """Normalize parameters as expected by Node construnctor argument."""
        def get_parameter_value(value):
            """Guess the desired type of the parameter based on the string value."""
            if Node._is_iterable(value):
                return [Node._get_parameter_value(item) for item in value]
            return _get_parameter_value(value)

        def _get_parameter_value(string_value):
            if string_value.lower() in ('true', 'false'):
                return string_value.lower() == 'true'
            if _is_integer(string_value):
                return int(string_value)
            if _is_float(string_value):
                return float(string_value)
            else:
                return string_value

        def _is_iterable(value):
            try:
                iter(value)
            except TypeError:
                return False
            return True

        def _is_integer(string_value):
            try:
                int(string_value)
            except ValueError:
                return False
            return True

        def _is_float(string_value):
            try:
                float(string_value)
            except ValueError:
                return False
            return True

        def get_nested_dictionary_from_nested_key_value_pairs(params):
            """Convert nested params in a nested dictionary."""
            # TODO(ivanpauno): If our schema checking is not powerfull enough
            # this could easily end in an infinite loop.
            # In that case, we should do some extra formal checking before processing.
            param_dict = {}
            for param in params:
                if hasattr(param, 'value'):
                    param_dict[param.name] = param.value
                else:
                    param_dict.update(
                        {param.name: get_nested_dictionary_from_nested_key_value_pairs(
                            param.param)})
            return param_dict

        if not params:
            return None
        normalized_params = []
        params_without_from = []
        for param in params:
            if hasattr(param, 'from'):
                # TODO(ivanpauno):
                # 'from' attribute ignores 'name' attribute,
                # it's not accepted to be nested,
                # and it can not have childs.
                # The first two things could be supported,
                # if 'SomeParameters' accept a file nested in
                # a dictionary as a value.
                normalized_params.append(param.__getattr__('from'))
                continue
            if hasattr(param, 'name'):
                params_without_from.append(param)
                continue
            raise RuntimeError('name or from attributes are needed')
        normalized_params.append(
            get_nested_dictionary_from_nested_key_value_pairs(params_without_from))
        return normalized_params

    @staticmethod
    def parse(entity: Entity):
        """Parse node."""
        package = entity.package
        executable = entity.executable
        name = getattr(entity, 'name', None)
        ns = getattr(entity, 'ns', None)
        prefix = getattr(entity, 'launch-prefix', None)
        output = getattr(entity, 'output', None)
        args = getattr(entity, 'args', None)
        args = args.split(' ') if args else None
        # TODO(ivanpauno): Add conditions to substitutions
        env = getattr(entity, 'env', None)
        if env is not None:
            env = {e.name: e.value for e in env}
        remappings = getattr(entity, 'remap', None)
        if remappings:
            remappings = [(remap.__getattr__('from'), remap.to) for remap in remappings]
        parameters = Node.parse_nested_parameters(getattr(entity, 'param', None))
        # TODO(ivanpauno): Handle if and unless attributes.

        return Node(
            package=package,
            node_executable=executable,
            node_name=name,
            node_namespace=ns,
            prefix=prefix,
            output=output,
            arguments=args,
            env=env,
            remappings=remappings,
            parameters=parameters)

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    def _create_params_file_from_dict(self, params):
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            # TODO(dhood): clean up generated parameter files.
            param_dict = {'/**': {'ros__parameters': params}}
            yaml.dump(param_dict, h, default_flow_style=False)
            return param_file_path

    def _perform_substitutions(self, context: LaunchContext) -> None:
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__expanded_node_name)
            self.__expanded_node_name.lstrip('/')
            if self.__node_namespace is not None:
                self.__expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            if not self.__expanded_node_namespace.startswith('/'):
                self.__expanded_node_namespace = '/' + self.__expanded_node_namespace
            validate_namespace(self.__expanded_node_namespace)
        except Exception:
            self.__logger.error(
                "Error while expanding or validating node name or namespace for '{}':"
                .format('package={}, node_executable={}, name={}, namespace={}'.format(
                    self.__package,
                    self.__node_executable,
                    self.__node_name,
                    self.__node_namespace,
                ))
            )
            raise
        self.__final_node_name = ''
        if self.__expanded_node_namespace not in ['', '/']:
            self.__final_node_name += self.__expanded_node_namespace
        self.__final_node_name += '/' + self.__expanded_node_name
        # expand parameters too
        if self.__parameters is not None:
            self.__expanded_parameter_files = []
            evaluated_parameters = evaluate_parameters(context, self.__parameters)
            for params in evaluated_parameters:
                if isinstance(params, dict):
                    param_file_path = self._create_params_file_from_dict(params)
                elif isinstance(params, pathlib.Path):
                    param_file_path = str(params)
                else:
                    raise RuntimeError('invalid normalized parameters {}'.format(repr(params)))
                if not os.path.isfile(param_file_path):
                    self.__logger.warning(
                        'Parameter file path is not a file: {}'.format(param_file_path),
                    )
                    # Don't skip adding the file to the parameter list since space has been
                    # reserved for it in the ros_specific_arguments.
                self.__expanded_parameter_files.append(param_file_path)
        # expand remappings too
        if self.__remappings is not None:
            self.__expanded_remappings = []
            for k, v in self.__remappings:
                key = perform_substitutions(context, normalize_to_list_of_substitutions(k))
                value = perform_substitutions(context, normalize_to_list_of_substitutions(v))
                self.__expanded_remappings.append((key, value))

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments = []  # type: List[Text]
        if self.__node_name is not None:
            ros_specific_arguments.append('__node:={}'.format(self.__expanded_node_name))
        if self.__node_namespace is not None:
            ros_specific_arguments.append('__ns:={}'.format(self.__expanded_node_namespace))
        if self.__expanded_parameter_files is not None:
            for param_file_path in self.__expanded_parameter_files:
                ros_specific_arguments.append('__params:={}'.format(param_file_path))
        if self.__expanded_remappings is not None:
            for remapping_from, remapping_to in self.__expanded_remappings:
                ros_specific_arguments.append('{}:={}'.format(remapping_from, remapping_to))
        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})
        return super().execute(context)
