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

from unittest.mock import patch

from launch_ros.ros_adapters import ROSAdapter


def test_ros_adapter_shutdown_releases_resources():
    adapter = ROSAdapter()
    old_context = adapter.ros_context
    old_executor = adapter.ros_executor

    with patch.object(
        adapter.ros_executor,
        'shutdown',
        wraps=adapter.ros_executor.shutdown,
    ) as executor_shutdown:
        adapter.shutdown()

    executor_shutdown.assert_called_once_with()
    assert adapter.ros_node is None

    adapter.start()
    assert adapter.ros_context is not old_context
    assert adapter.ros_node is not None
    assert adapter.ros_executor is not old_executor
    adapter.shutdown()
