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

"""Example of how to parse an xml."""

from pathlib import Path

from launch import LaunchService

from launch_frontend import Entity, parse_description


def test_node_xml_frontend():
    """Parse node xml example."""
    root_entity = Entity.load(str(Path(__file__).parent / 'node.xml'))
    ld = parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert(0 == ls.run())


if __name__ == '__main__':
    test_node_xml_frontend()
