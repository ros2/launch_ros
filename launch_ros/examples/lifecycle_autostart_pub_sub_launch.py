# Copyright 2024 Open Navigation LLC
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

"""Launch a lifecycle talker and a lifecycle listener."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch.events  # noqa: E402

import launch_ros.actions  # noqa: E402
import launch_ros.events  # noqa: E402
import launch_ros.events.lifecycle  # noqa: E402


def main(argv=sys.argv[1:]):
    ld = launch.LaunchDescription()
    talker_node = launch_ros.actions.LifecycleNode(
        name='talker',
        namespace='',
        package='lifecycle',
        executable='lifecycle_talker',
        output='screen',
        autostart=True)
    listener_node = launch_ros.actions.LifecycleNode(
        name='listener',
        namespace='',
        package='lifecycle',
        executable='lifecycle_listener',
        output='screen',
        autostart=True)
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
