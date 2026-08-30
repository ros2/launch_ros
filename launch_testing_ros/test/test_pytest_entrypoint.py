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

import launch_testing_ros_pytest_entrypoint
import pytest


class LegacyLaunchTestingHooks:

    @pytest.hookspec(firstresult=True)
    def pytest_launch_collect_makemodule(path, parent, entrypoint):
        pass


class CurrentLaunchTestingHooks:

    @pytest.hookspec(firstresult=True)
    def pytest_launch_collect_makemodule(module_path, path, parent, entrypoint):
        pass


@pytest.mark.parametrize(
    'hookspecs', [LegacyLaunchTestingHooks, CurrentLaunchTestingHooks]
)
def test_pytest_entrypoint_hook_compatibility(hookspecs):
    manager = pytest.PytestPluginManager()
    manager.add_hookspecs(hookspecs)
    manager.register(launch_testing_ros_pytest_entrypoint, 'launch_ros')
