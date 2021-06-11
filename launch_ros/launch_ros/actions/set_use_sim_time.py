# Copyright 2021 Open Source Robotics Foundation, Inc.
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


from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext

from rclpy.parameter import Parameter
from launch_ros.ros_adapters import get_ros_node


@expose_action('use_sim_time_action')
class SetUseSimTime(Action):
    """Action that sets the 'use_sim_time' parameter in the current context."""

    def __init__(
        self,
        value: bool,
        **kwargs
    ) -> None:
        """Create a UseSimTimeAction."""
        super().__init__(**kwargs)
        self.__value = value

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `UseSimTime` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['value'] = parser.parse_substitution(entity.get_attr('value'))
        return cls, kwargs

    @property
    def value(self) -> bool:
        """Getter for value."""
        return self.__value

    def execute(self, context: LaunchContext):
        """Execute the action."""
        node = get_ros_node(context)
        param = Parameter(
            'use_sim_time',
            Parameter.Type.BOOL,
            self.value
        )
        node.set_parameters([param])
