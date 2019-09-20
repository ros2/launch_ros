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

"""Test find-package-share substitution."""

import io
import textwrap
from typing import List
from typing import Optional

from ament_index_python.packages import get_package_share_directory

from launch import Action
from launch import LaunchContext
from launch import LaunchDescriptionEntity
from launch import LaunchService
from launch import SomeSubstitutionsType
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

import pytest


@expose_action('test_find_frontend')
class CheckSubstitutionAction(Action):

    def __init__(self, subst: SomeSubstitutionsType, result: SomeSubstitutionsType, **kwargs):
        self.subst = normalize_to_list_of_substitutions(subst)
        self.result = normalize_to_list_of_substitutions(result)
        super().__init__(**kwargs)

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        assert perform_substitutions(context, self.subst) == \
            perform_substitutions(context, self.result)

    @classmethod
    def parse(cls, entity, parser):
        _, kwargs = super().parse(entity, parser)
        kwargs['subst'] = parser.parse_substitution(entity.get_attr('subst'))
        kwargs['result'] = parser.parse_substitution(entity.get_attr('result'))
        return cls, kwargs


result = get_package_share_directory('launch') + '/sub/path/foo'

xml_file = \
    r"""
    <launch>
        <test_find_frontend subst="$(find-pkg-share launch)/sub/path/foo" result="{}"/>
    </launch>
    """.format(result)
xml_file = textwrap.dedent(xml_file)

yaml_file = \
    r"""
    launch:
    - test_find_frontend:
        subst: '$(find-pkg-share launch)/sub/path/foo'
        result: '{}'
    """.format(result)
xml_file = textwrap.dedent(xml_file)


@pytest.mark.parametrize('file', (xml_file, yaml_file))
def test_node_frontend(file):
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(file))
    ld = parser.parse_description(root_entity)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert(0 == ls.run())
