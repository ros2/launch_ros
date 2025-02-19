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

import sys

import launch  # noqa: E402

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableLifecycleNode


def main(argv=sys.argv[1:]):
    ld = launch.LaunchDescription()

    # Can autostart from the container
    container = ComposableNodeContainer(
            name='lifecycle_component_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableLifecycleNode(
                    package='composition',
                    plugin='composition::Listener',
                    name='listener',
                    autostart=True),
                ]
            )

    # ... and also from an external loader
    loader = LoadComposableNodes(
        target_container='lifecycle_component_container',
        composable_node_descriptions=[
            ComposableLifecycleNode(
                package='composition',
                plugin='composition::Talker',
                name='talker',
                autostart=True),
        ],
    )

    ld.add_action(container)
    ld.add_action(loader)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
