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

import contextlib
import os
import threading
from typing import List
from typing import Optional

import launch
import launch.events

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future


class ROSAdapter:
    """Wraps rclpy API to ease ROS node usage in `launch_ros` actions."""

    def __init__(
        self,
        *,
        argv: Optional[List[str]] = None,
        autostart: bool = True
    ):
        """
        Construct adapter.

        :param: argv List of global arguments for rclpy context initialization.
        :param: autostart Whether to start adapter on construction or not.
        """
        # Do not use `None` here, as `rclpy.init` will use `sys.argv` in that case.
        self.__argv = [] if argv is None else argv
        self.__ros_context = None
        self.__ros_node = None
        self.__ros_executor = None
        self.__is_running = False
        # Serializes construction of the executor's wait set (in the spin
        # thread, see `_run`) with creation/destruction of entities
        # (subscriptions, clients, services, timers, ...) on `__ros_node` from
        # other threads. Mutating the node's entities while the executor builds
        # its wait set races at the rcl layer and can crash the process; this
        # has been observed as an access violation on Windows.
        #
        # `__node_lock` guards the actual critical section. `__waiters_cv` and
        # `__waiters` implement fairness: the spin thread would otherwise hold
        # `__node_lock` almost continuously (re-acquiring it before a blocked
        # thread is scheduled), starving entity creation. The spin thread yields
        # `__node_lock` whenever a thread is waiting to access the node.
        # `__spin_interrupt` is a per-iteration future a waiting thread completes
        # to cut the current `spin_once` short instead of waiting for its full
        # timeout.
        self.__node_lock = threading.Lock()
        self.__waiters_cv = threading.Condition()
        self.__waiters = 0
        self.__spin_interrupt = None

        if autostart:
            self.start()

    def start(self):
        """Start ROS adapter."""
        if self.__is_running:
            raise RuntimeError('Cannot start a ROS adapter that is already running')
        self.__ros_context = rclpy.Context()
        rclpy.init(args=self.__argv, context=self.__ros_context)
        self.__ros_node = rclpy.create_node(
            'launch_ros_{}'.format(os.getpid()),
            context=self.__ros_context
        )
        self.__ros_executor = SingleThreadedExecutor(context=self.__ros_context)
        self.__is_running = True

        self.__ros_executor_thread = threading.Thread(target=self._run)
        self.__ros_executor_thread.start()

    def _run(self):
        try:
            self.__ros_executor.add_node(self.__ros_node)
            while self.__is_running:
                interrupt = Future(executor=self.__ros_executor)
                # Defer to any thread waiting to create/destroy entities on the
                # node, so it does not race with the wait set built below and is
                # not starved by this loop re-acquiring `__node_lock`. Then
                # publish the interrupt future so a thread that starts waiting
                # once we are spinning can cut the spin short.
                with self.__waiters_cv:
                    self.__waiters_cv.wait_for(
                        lambda: self.__waiters == 0 or not self.__is_running)
                    self.__spin_interrupt = interrupt
                # TODO(wjwwood): switch this to `spin()` when it considers
                #   asynchronously added subscriptions.
                #   see: https://github.com/ros2/rclpy/issues/188
                with self.__node_lock:
                    self.__ros_executor.spin_once_until_future_complete(
                        interrupt, timeout_sec=1.0)
                with self.__waiters_cv:
                    self.__spin_interrupt = None
        except KeyboardInterrupt:
            pass
        finally:
            self.__ros_executor.remove_node(self.__ros_node)

    def shutdown(self):
        """Shutdown ROS adapter."""
        if not self.__is_running:
            raise RuntimeError('Cannot shutdown a ROS adapter that is not running')
        self.__is_running = False
        # Wake the spin thread in case it is parked in the fairness gate.
        with self.__waiters_cv:
            self.__waiters_cv.notify_all()
        self.__ros_executor_thread.join()
        self.__ros_node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)

    @property
    def argv(self):
        return self.__argv

    @property
    def ros_context(self):
        return self.__ros_context

    @property
    def ros_node(self):
        return self.__ros_node

    @property
    def ros_executor(self):
        return self.__ros_executor

    @contextlib.contextmanager
    def node_access(self):
        """
        Serialize access to the managed node with the executor spin thread.

        Use this context manager around any creation or destruction of entities
        (subscriptions, clients, services, timers, ...) on :attr:`ros_node`::

            with ros_adapter.node_access() as node:
                node.create_client(...)

        It serializes those mutations with the executor thread as it builds its
        wait set in :meth:`_run`, which would otherwise race at the rcl layer
        and can crash the process (e.g. an access violation on Windows).
        """
        # Announce intent so the spin thread yields `__node_lock` to us instead
        # of continuously re-acquiring it, then complete the interrupt future so
        # the current `spin_once` returns promptly rather than blocking for the
        # full timeout.
        with self.__waiters_cv:
            self.__waiters += 1
            interrupt = self.__spin_interrupt
        if interrupt is not None and not interrupt.done():
            interrupt.set_result(None)
        self.__node_lock.acquire()
        try:
            yield self.__ros_node
        finally:
            self.__node_lock.release()
            with self.__waiters_cv:
                self.__waiters -= 1
                self.__waiters_cv.notify_all()


def get_ros_adapter(context: launch.LaunchContext):
    """
    Get the ROS adapter managed by the given launch context.

    If no adapter is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    if not hasattr(context.locals, 'ros_adapter'):
        ros_adapter = ROSAdapter()
        context.extend_globals({'ros_adapter': ros_adapter})
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=lambda *args, **kwargs: ros_adapter.shutdown()
        ))
    return context.locals.ros_adapter


def get_ros_node(context: launch.LaunchContext):
    """
    Get the ROS node managed by the given launch context.

    If no node is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    return get_ros_adapter(context).ros_node
