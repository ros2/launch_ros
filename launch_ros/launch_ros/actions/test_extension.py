# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from launch_ros.actions.node import NodeActionExtension

class TestNodeActionExtension(NodeActionExtension):
    def command_extension(self, context):
        print(f'command_extension called with arguments:\ncontext:\t{context}')
        return []

    def pre_execute(self, context, ros_specific_arguments, node_action):
        print(f'execute called with arguments:\ncontext:\t{context}\nros_specific_arguments\t{ros_specific_arguments}\nnode_action\t{node_action}')
        ros_specific_arguments['__test_arg'] = 'test_value'
        return ros_specific_arguments
