# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from typing import List
from typing import Optional

from launch.action import Action
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.some_entities_type import SomeEntitiesType
from rmw_test_fixture_implementation import rmw_test_isolation_start
from rmw_test_fixture_implementation import rmw_test_isolation_stop


class EnableRmwIsolation(Action):
    """Action which enables isolation of ROS communication using rmw_test_fixture."""

    def __init__(self, **kwargs) -> None:
        """Create a EnableRmwIsolation action."""
        super().__init__(**kwargs)

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeEntitiesType]:
        rmw_test_isolation_stop()

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        rmw_test_isolation_start()
        context.register_event_handler(OnShutdown(
            on_shutdown=self.__on_shutdown))
