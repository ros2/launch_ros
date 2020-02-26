# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    path_to_fixtures = os.path.dirname(__file__)
    path_to_talker_node_script = os.path.join(path_to_fixtures, 'talk_once_node.py')
    path_to_listener_node_script = os.path.join(path_to_fixtures, 'listen_once_node.py')

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            node_executable=sys.executable,
            arguments=[path_to_talker_node_script],
            node_name='talker',
            output='screen',
            emulate_tty=True
        ),
        launch_ros.actions.Node(
            node_executable=sys.executable,
            arguments=[path_to_listener_node_script],
            node_name='listener',
            output='screen',
            emulate_tty=True
        )
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
