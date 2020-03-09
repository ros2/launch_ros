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

"""Module containing ROS specific adapters and their initialization."""

import os
import threading

import launch
import launch.events

import rclpy
from rclpy.executors import SingleThreadedExecutor


class ROSAdapter:
    """Wraps rclpy API to ease ROS node usage in `launch_ros` actions."""

    def __init__(
        self,
        *,
        argv: Optional[List[str]]=None,
        context: Optional[rclpy.Context]=None,
        autostart: bool=True
    ):
        """
        :param: argv List of global arguments for rclpy context initialization.
        :param: context Provide a context other than the default rclpy context
          to pass down to rclpy.init.
          The context is expected to have already been initialized by the caller
          using rclpy.init().
        :param: autostart Whether to start adapter on construction or not.
        """
        self.__argv = argv
        self.__context = context
        self.__node = None
        self.__executor = None
        self.__is_running = False

        if autostart:
            self.start()

    def start(self):
        """Start ROS adapter."""
        if self.__is_running:
            raise RuntimeError('Cannot start a ROS adapter that is already running')
        try:
            if self.__context is not None:
                rclpy.init(args=self.__argv, context=self.__context)
        except RuntimeError as exc:
            if 'rcl_init called while already initialized' in str(exc):
                pass
            raise
        self.__node = rclpy.create_node(
            'launch_ros_{}'.format(os.getpid()),
            context=self.__context
        )
        self.__is_running = True
        self.__executor = SingleThreadedExecutor(context=self.__context)
        self.__executor_thread = threading.Thread(target=self._run)
        self.__executor_thread.start()

    def _run(self):
        try:
            self.__executor.add_node(self.__node)
            while self.__is_running:
                # TODO(wjwwood): switch this to `spin()` when it considers
                #   asynchronously added subscriptions.
                #   see: https://github.com/ros2/rclpy/issues/188
                executor.spin_once(timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.__executor.remove_node(self.__node)

    def shutdown(self):
        """Shutdown ROS adapter."""
        if not self.__is_running:
            raise RuntimeError('Cannot shutdown a ROS adapter that is not running')
        self.__executor_thread.join()
        self.__node.destroy_node()
        rclpy.shutdown(context=self.__context)
        self.__is_running = False

    @property
    def argv(self):
        return self.__argv

    @property
    def context(self):
        return self.__context

    @property
    def node(self):
        return self.__node

    @property
    def executor(self):
        return self.__executor


def get_ros_adapter(*, context: launch.LaunchContext):
    """
    Get the ROS adapter managed by the given launch context.

    If no adapter is found, one will be created.
    """
    if not hasattr(context.locals, 'ros_adapter'):
        ros_adapter = ROSAdapter()
        context.extend_globals({'ros_adapter': ros_adapter})
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=lambda *args, **kwargs: ros_adapter.shutdown()
        ))
    return context.locals['ros_adapter']


def get_ros_node(*, context: launch.LaunchContext):
    """
    Get the ROS node managed by the given launch context.

    If no node is found, one will be created.
    """
    return get_ros_adapter(context).node
