# Copyright 2026 ktyang512
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

import threading

from launch_ros.ros_adapters import ROSAdapter

import pytest

from rclpy.executors import ShutdownException


def test_ros_adapter_shutdown_releases_resources():
    adapter = ROSAdapter()
    try:
        old_context = adapter.ros_context
        old_node = adapter.ros_node
        old_executor = adapter.ros_executor

        fired = threading.Event()
        old_node.create_timer(0.001, fired.set)
        assert fired.wait(timeout=5.0)
    finally:
        adapter.shutdown()

    # spin_once() catches ShutdownException, so check callback retrieval directly.
    with pytest.raises(ShutdownException):
        old_executor.wait_for_ready_callbacks(timeout_sec=0)

    assert adapter.ros_node is None

    adapter.start()
    try:
        assert adapter.ros_context is not old_context
        assert adapter.ros_node is not None
        assert adapter.ros_node is not old_node
        assert adapter.ros_executor is not old_executor
    finally:
        adapter.shutdown()
