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

import argparse

from ros2launch.command.launch import LaunchCommand


def test_launch_prefix_alias():
    parser = argparse.ArgumentParser()
    cmd = LaunchCommand()
    cmd.add_arguments(parser, 'ros2 launch')
    args = parser.parse_args(
        ['--prefix', 'gdb -ex run --args', 'my_pkg', 'my_launch.py'])
    assert args.launch_prefix == 'gdb -ex run --args'


def test_launch_prefix_filter_alias():
    parser = argparse.ArgumentParser()
    cmd = LaunchCommand()
    cmd.add_arguments(parser, 'ros2 launch')
    args = parser.parse_args(
        ['--prefix-filter', 'my_node', 'my_pkg', 'my_launch.py'])
    assert args.launch_prefix_filter == 'my_node'
