# Copyright 2018-2020 Open Source Robotics Foundation, Inc.
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

import launch.actions
import launch_ros.ros_adapters

import rclpy


class ExplicitROSStartup(launch.actions.OpaqueFunction):
    """Does ROS specific launch startup explicitly."""

    def __init__(
        self,
        *,
        argv: Optional[List[str]] = None,
        rclpy_context: Optional[rclpy.Context] = None
    ):
        """
        Create a ExplicitROSStartup opaque function.

        :param: argv List of global arguments for rclpy context initialization.
        :param: rclpy_context Provide a context other than the default rclpy context
          to pass down to rclpy.init.
          The context is expected to have already been initialized by the caller
          using rclpy.init().
        """
        super().__init__(function=self._function)
        self.__ros_adapter = launch_ros.ros_adapters.ROSAdapter(
            argv=argv, context=rclpy_context, autostart=False
        )

    def _shutdown(self, event: launch.Event, context: launch.LaunchContext):
        self.__ros_adapter.shutdown()

    def _function(self, context: launch.LaunchContext):
        if hasattr(context.locals, 'ros_adapter'):
            raise RuntimeError(
                'A ROS adapter is already been managed by this '
                'launch context. Duplicate explicit ROS startup?'
            )
        context.extend_globals({'ros_adapter': self.__ros_adapter})
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=self._shutdown
        ))
        self.__ros_adapter.start()
