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

# Only import standard python modules here
# This module intentionally delays imports as late as possible to avoid
# importing downstream modules in upstream packages when built with a merged
# workspace.

import pathlib

import pytest


def pytest_launch_collect_makemodule(path, parent, entrypoint):
    marks = getattr(entrypoint, 'pytestmark', [])
    if marks and any(m.name == 'rostest' for m in marks):
        from launch_testing_ros.pytest.hooks import LaunchROSTestModule
<<<<<<< HEAD
        module = LaunchROSTestModule.from_parent(parent=parent, fspath=path)
=======
        if _pytest_version_ge(7):
            path = pathlib.Path(module_path)
            module = LaunchROSTestModule.from_parent(parent=parent, path=path)
        else:
            module = LaunchROSTestModule.from_parent(parent=parent, fspath=module_path)
>>>>>>> 1c0a7ae (Fix launch_testing_ros so it works with pytest 7. (#543))
        for mark in marks:
            decorator = getattr(pytest.mark, mark.name)
            decorator = decorator.with_args(*mark.args, **mark.kwargs)
            module.add_marker(decorator)
        return module


def pytest_configure(config):
    config.addinivalue_line(
        'markers',
        'rostest: mark a generate_test_description function as a ROS launch test entrypoint'
    )
