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

"""Module for the `SetROSLogDir` action."""

import os
from typing import List

from launch import Action
from launch import Substitution
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from rclpy.logging import get_logging_directory


@expose_action('set_ros_log_dir')
class SetROSLogDir(Action):
    """
    Action that sets the ros log directory.

    This is done by setting the ROS_LOG_DIR environment variable, which will
    influence all processes started after that time, in which ros was initialized.

    This can be used in combination with launch.actions.GroupAction and its
    ``scoped=true`` option to provide scoped changes to this environment
    variable.

    Note this will not affect nodes loaded into component containers which were
    started before this action is executed.
    """

    def __init__(
        self,
        new_log_dir: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """
        Create a SetROSLogDir action.

        :param new_log_dir: new log directory, if absolute it sets the ros log dir,
            but if it is a relative path, then it is joined with the current ros log dir.
        """
        super().__init__(**kwargs)
        self.__log_dir = normalize_to_list_of_substitutions(new_log_dir)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetROSLogDir` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['new_log_dir'] = parser.parse_substitution(entity.get_attr('new_log_dir'))
        return cls, kwargs

    @property
    def log_dir(self) -> List[Substitution]:
        """Getter for self.__log_dir."""
        return self.__log_dir

    def execute(self, context: LaunchContext):
        """Execute the action."""
        new_log_dir = perform_substitutions(context, self.log_dir)
        current_rclpy_logging_directory = get_logging_directory()
        # Prefer ROS_LOG_DIR over what rclpy reports, but fall back to that if not set.
        current_log_dir = os.environ.get('ROS_LOG_DIR', current_rclpy_logging_directory)
        # If new_log_dir is abs, then current_log_dir will be truncated and not used.
        log_dir = os.path.join(current_log_dir, new_log_dir)
        assert os.path.isabs(log_dir)
        os.environ['ROS_LOG_DIR'] = log_dir
