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

"""Utilities for ``ros2 launch --dump-params`` dry-run parameter collection."""

from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple

from launch.launch_context import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

import yaml

from .evaluate_parameters import evaluate_parameters
from .namespace_utils import prefix_namespace
from .normalize_parameters import normalize_parameter_dict
from .to_parameters_list import to_parameters_list

from ..parameter_descriptions import ParameterFile

DUMP_PARAMS_CONTEXT_KEY = 'dump_params_collector'


class DumpParamsError(RuntimeError):
    """Raised when parameter dumping cannot produce a reliable result."""


class DumpParamsCollector:
    """Collects resolved per-node parameter dicts during a dry-run launch."""

    def __init__(self) -> None:
        # fqn -> params dict (insertion order preserved for stable YAML)
        self._nodes: Dict[str, Dict[str, Any]] = {}
        self._warnings: List[str] = []

    @property
    def warnings(self) -> List[str]:
        return list(self._warnings)

    def warn(self, message: str) -> None:
        self._warnings.append(message)

    def add_node(self, fqn: str, params: Mapping[str, Any]) -> None:
        """
        Record parameters for a fully qualified node name.

        Identical param sets for the same FQN are coalesced. Divergent sets raise.
        """
        normalized_fqn = _normalize_fqn(fqn)
        param_dict = dict(params)
        if normalized_fqn in self._nodes:
            if self._nodes[normalized_fqn] == param_dict:
                return
            raise DumpParamsError(
                "duplicate node '{}' with divergent parameter sets".format(normalized_fqn)
            )
        self._nodes[normalized_fqn] = param_dict

    def to_yaml_dict(self) -> Dict[str, Any]:
        """Return the standard params-file mapping."""
        return {
            fqn: {'ros__parameters': params}
            for fqn, params in self._nodes.items()
        }

    def dumps(self) -> str:
        """Serialize collected parameters as a params-file YAML document."""
        return yaml.safe_dump(
            self.to_yaml_dict(),
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True,
        )


def is_dump_params_mode(context: LaunchContext) -> bool:
    """Return True when the launch context is collecting dump-params output."""
    try:
        return getattr(context.locals, DUMP_PARAMS_CONTEXT_KEY) is not None
    except AttributeError:
        return False


def get_dump_params_collector(context: LaunchContext) -> Optional[DumpParamsCollector]:
    """Return the collector attached to the context, if any."""
    try:
        return getattr(context.locals, DUMP_PARAMS_CONTEXT_KEY)
    except AttributeError:
        return None


def attach_dump_params_collector(
    context: LaunchContext,
    collector: Optional[DumpParamsCollector] = None,
) -> DumpParamsCollector:
    """Attach a collector to the launch context and return it."""
    if collector is None:
        collector = DumpParamsCollector()
    context.extend_globals({DUMP_PARAMS_CONTEXT_KEY: collector})
    return collector


def resolve_scoped_parameters(
    context: LaunchContext,
    *,
    node_name: str,
    namespace: Optional[str],
    normalized_parameters: Optional[Iterable[Any]] = None,
    ros_arguments: Optional[Iterable[Any]] = None,
) -> Dict[str, Any]:
    """
    Resolve the effective parameter dict for a node, matching runtime merge order.

    Order:
      1. SetParameter / SetParametersFromFile (global_params)
      2. the node's parameters=[...] list
      3. -p / --param overrides from ros_arguments
    """
    parameters: List[Any] = []
    params_container = context.launch_configurations.get('global_params', None)
    if params_container is not None:
        for param in params_container:
            if isinstance(param, tuple):
                parameters.append(normalize_parameter_dict({param[0]: param[1]}))
            else:
                param_file_path = Path(param).resolve()
                parameters.append(ParameterFile(param_file_path))
    if normalized_parameters:
        parameters.extend(list(normalized_parameters))

    result: Dict[str, Any] = {}
    if parameters:
        param_list = to_parameters_list(
            context,
            node_name,
            namespace or '',
            evaluate_parameters(context, parameters),
        )
        for parameter in param_list:
            result[parameter.name] = _normalize_yaml_value(parameter.value)

    for name, value in _param_overrides_from_ros_arguments(context, ros_arguments):
        result[name] = value
    return result


def apply_node_remaps_to_fqn(
    fqn: str,
    remappings: Optional[Iterable[Tuple[str, str]]],
) -> str:
    """Apply ``__node`` / ``__ns`` remappings to a fully qualified node name."""
    normalized = _normalize_fqn(fqn)
    parts = [p for p in normalized.split('/') if p]
    node_name = parts[-1] if parts else ''
    namespace = '/' + '/'.join(parts[:-1]) if len(parts) > 1 else '/'
    if not remappings:
        return _normalize_fqn(prefix_namespace(namespace, node_name) or node_name)
    for src, dst in remappings:
        if src == '__node':
            node_name = dst.lstrip('/')
        elif src == '__ns':
            namespace = dst if dst.startswith('/') else '/' + dst
    combined = prefix_namespace(namespace, node_name)
    return _normalize_fqn(combined or node_name)


def _param_overrides_from_ros_arguments(
    context: LaunchContext,
    ros_arguments: Optional[Iterable[Any]],
) -> List[Tuple[str, Any]]:
    if not ros_arguments:
        return []
    expanded: List[str] = []
    for arg in ros_arguments:
        expanded.append(
            perform_substitutions(context, normalize_to_list_of_substitutions(arg))
        )
    overrides: List[Tuple[str, Any]] = []
    i = 0
    while i < len(expanded):
        token = expanded[i]
        if token in ('-p', '--param'):
            if i + 1 >= len(expanded):
                break
            overrides.append(_parse_param_rule(expanded[i + 1]))
            i += 2
            continue
        if token.startswith('-p') and token not in ('-p',):
            # Unsupported glued form; ignore for MVP.
            i += 1
            continue
        i += 1
    return overrides


def _parse_param_rule(rule: str) -> Tuple[str, Any]:
    if ':=' not in rule:
        raise DumpParamsError("invalid -p override '{}', expected name:=value".format(rule))
    name, raw_value = rule.split(':=', maxsplit=1)
    try:
        value = yaml.safe_load(raw_value)
    except yaml.YAMLError:
        value = raw_value
    return name, _normalize_yaml_value(value)


def _normalize_fqn(fqn: str) -> str:
    if not fqn:
        return '/'
    if not fqn.startswith('/'):
        return '/' + fqn
    return fqn


def _normalize_yaml_value(value: Any) -> Any:
    if isinstance(value, tuple):
        return list(value)
    return value
