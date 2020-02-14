# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Test type checking/coercion utils."""

from launch import LaunchContext
from launch.substitutions import TextSubstitution

from launch_ros.descriptions import Parameter

import pytest


def test_with_scalar():
    lc = LaunchContext()

    param = Parameter(name='my_param', value='asd')
    assert isinstance(param.name, list)
    assert len(param.name) == 1
    assert isinstance(param.name[0], TextSubstitution)
    assert param.name[0].text == 'my_param'
    assert param.value == 'asd'
    assert param.value_type is None
    assert param.evaluate(lc) == ('my_param', 'asd')
    # After the first `evaluate` call, the followings `.name` `.value` and `.evaluate()`
    # calls are calculated differently. Test them too.
    assert param.name == 'my_param'
    assert param.value == 'asd'
    assert param.evaluate(lc) == ('my_param', 'asd')

    param = Parameter(name='my_param', value='asd', value_type=str)
    assert isinstance(param.name, list)
    assert len(param.name) == 1
    assert isinstance(param.name[0], TextSubstitution)
    assert param.name[0].text == 'my_param'
    assert param.value == 'asd'
    assert param.value_type is str
    assert param.evaluate(lc) == ('my_param', 'asd')
    # After the first `evaluate` call, the followings `.name` `.value` and `.evaluate()`
    # calls are calculated differently. Test them too.
    assert param.name == 'my_param'
    assert param.value == 'asd'
    assert param.evaluate(lc) == ('my_param', 'asd')


def test_value_is_substitution():
    lc = LaunchContext()

    param = Parameter(name='my_param', value=TextSubstitution(text='1'))
    assert isinstance(param.value, TextSubstitution)
    assert param.evaluate(lc) == ('my_param', 1)

    param = Parameter(
        name='my_param',
        value=[
            '[',
            TextSubstitution(text='1, '),
            TextSubstitution(text='2, '),
            TextSubstitution(text='3, '),
            ']',
        ]
    )
    assert isinstance(param.value, list)
    assert param.evaluate(lc) == ('my_param', [1, 2, 3])

    param = Parameter(
        name='my_param',
        value=TextSubstitution(text='[1, 2, 3]')
    )
    assert isinstance(param.value, TextSubstitution)
    assert param.evaluate(lc) == ('my_param', [1, 2, 3])


def test_evaluate_fails():
    lc = LaunchContext()

    param = Parameter(name='my_param', value='1', value_type=int)
    with pytest.raises(ValueError):
        param.evaluate(lc)

    param = Parameter(
        name='my_param',
        value=TextSubstitution(text='[1, asd, 3]')
    )
    with pytest.raises(ValueError):
        param.evaluate(lc)
