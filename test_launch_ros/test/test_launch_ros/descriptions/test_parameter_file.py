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

"""Tests for launch_ros.descriptions.ParameterFile"""

import os
from contextlib import contextmanager
from tempfile import NamedTemporaryFile

from launch_ros.description import ParameterFile

@contextmanager
def get_parameter_file(contents, mode='w'):
    f = NamedTemporaryFile(delete=False, mode=mode)
    try:
        with f:
            f.write(contents)
        yield f.name
    finally:
        os.unlink(f.name)


class CustomSubstitution(Substitution):

    def __init__(self, text):
        self.__text = text

    def perform(self, context):
        return self.__text


@expose_substitution('test')
def parse_test_substitution(data):
    if not data or len(data) > 1:
        raise RuntimeError()
    kwargs = {}
    kwargs['text'] = perform_substitutions_without_context(data[0])
    return CustomSubstitution, kwargs



def get_test_cases():
    parameter_file_without_substitutions = (
        """\
/my_ns/my_node:
    ros__parameters:
        my_int: 1,
        my_str: '1',
        my_list_of_strs: ['1', '2', '3']
        """
    )
    parameter_file_with_substitutions = (
        """\
/$(test my_ns)/$(test my_node):
    ros__parameters:
        my_int: $(test 1),
        my_str: '1',
        my_list_of_strs: ['1', '2', '3']
        """
    )
