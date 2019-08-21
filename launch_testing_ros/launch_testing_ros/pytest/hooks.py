# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from launch_testing.pytest.hooks import LaunchTestItem
from launch_testing.pytest.hooks import LaunchTestModule

from ..test_runner import LaunchTestRunner


class LaunchROSTestItem(LaunchTestItem):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs, runner_cls=LaunchTestRunner)


class LaunchROSTestModule(LaunchTestModule):

    def makeitem(self, *args, **kwargs):
        return LaunchROSTestItem(*args, **kwargs)


def pytest_launch_collect_makemodule(path, parent, entrypoint):
    marks = getattr(entrypoint, 'pytestmark', [])
    if marks and any(m.name == 'rostest' for m in marks):
        return LaunchROSTestModule(path, parent)


def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'rostest: mark a generate_test_description function as a ROS launch test entrypoint'
    )
