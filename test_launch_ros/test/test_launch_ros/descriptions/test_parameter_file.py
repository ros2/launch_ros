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

"""Tests for launch_ros.descriptions.ParameterFile."""

from contextlib import contextmanager
import gc
import os
import sys
from tempfile import NamedTemporaryFile

from launch import Substitution
from launch.frontend import expose_substitution
from launch.utilities import perform_substitutions

from launch_ros.descriptions import ParameterFile

import pytest


@contextmanager
def get_parameter_file(contents, mode='w'):
    f = NamedTemporaryFile(delete=False, mode=mode)
    try:
        with f:
            f.write(contents)
        yield f.name
    finally:
        os.unlink(f.name)


class MockContext:

    def perform_substitution(self, sub):
        return sub.perform(None)


class CustomSubstitution(Substitution):

    def __init__(self, text):
        self.__text = text

    def perform(self, context):
        return self.__text

    def set_text(self, text):
        self.__text = text


@expose_substitution('test')
def parse_test_substitution(data):
    if not data or len(data) > 1:
        raise RuntimeError()
    kwargs = {}
    kwargs['text'] = perform_substitutions(MockContext(), data[0])
    return CustomSubstitution, kwargs


def get_test_cases():
    parameter_file_without_substitutions = (
        """\
/my_ns/my_node:
    ros__parameters:
        my_int: 1
        my_str: '1'
        my_list_of_strs: ['1', '2', '3']
        """
    )
    parameter_file_with_substitutions = (
        """\
'/$(test my_ns)/$(test my_node)':
    ros__parameters:
        my_int: '$(test 1)'
        my_str: '"$(test 1)"'
        my_list_of_strs: ['1', '"$(test 2)"', '3']
        """
    )
    return [
        pytest.param(
            parameter_file_with_substitutions,  # original contents
            parameter_file_without_substitutions,  # expected contents
            True,  # substitutions allowed
            id='Parameter file with substitutions, substitutions allowed',
        ),
        pytest.param(
            parameter_file_with_substitutions,
            parameter_file_with_substitutions,
            False,
            id='Parameter file with substitutions, substitutions not allowed',
        ),
        pytest.param(
            parameter_file_without_substitutions,
            parameter_file_without_substitutions,
            True,
            id='Parameter file without substitutions, substitutions allowed',
        ),
        pytest.param(
            parameter_file_without_substitutions,
            parameter_file_without_substitutions,
            False,
            id='Parameter file without substitutions, substitutions not allowed',
        ),
    ]


@pytest.mark.parametrize(
    'original_contents, expected_contents, allow_substs',
    get_test_cases(),
)
def test_parameter_file_description(original_contents, expected_contents, allow_substs):
    lc = MockContext()
    with get_parameter_file(original_contents) as file_name:
        desc = ParameterFile(file_name, allow_substs=allow_substs)
        if isinstance(desc.param_file, list):
            assert perform_substitutions(lc, desc.param_file) == file_name
        else:
            assert desc.param_file == file_name
        assert desc.allow_substs == allow_substs
        evaluated_param_file = desc.evaluate(lc)
        with open(evaluated_param_file, 'r') as new_f:
            new_f.read() == expected_contents
        assert desc.param_file == evaluated_param_file
        if not allow_substs:
            assert os.fspath(desc.param_file) == os.fspath(file_name)
        param_file = desc.param_file
        desc.cleanup()
        if allow_substs:
            assert not param_file.exists()
            assert isinstance(desc.param_file, list)
        else:
            assert param_file.exists()
            assert os.fspath(desc.param_file) == os.fspath(file_name)


def test_parameter_file_allow_substs_substitution():
    lc = MockContext()
    with get_parameter_file('{}') as file_name:
        desc = ParameterFile(
            file_name,
            allow_substs=CustomSubstitution('true'),
        )
        assert isinstance(desc.allow_substs, list)
        desc.evaluate(lc)
        assert desc.allow_substs is True
        desc.cleanup()
        assert isinstance(desc.allow_substs, list)


def test_parameter_file_invalid_allow_substs_does_not_fail_cleanup():
    unraisable = []
    original_unraisablehook = sys.unraisablehook
    sys.unraisablehook = unraisable.append
    try:
        with pytest.raises(TypeError, match='allow_subst'):
            ParameterFile('params.yaml', allow_substs=object())
        gc.collect()
    finally:
        sys.unraisablehook = original_unraisablehook

    assert not unraisable


@pytest.mark.parametrize('first, second', [('false', 'true'), ('true', 'false')])
def test_parameter_file_allow_substs_re_evaluates_after_cleanup(first, second):
    lc = MockContext()
    allow_substs = CustomSubstitution(first)
    with get_parameter_file('{}') as file_name:
        desc = ParameterFile(file_name, allow_substs=allow_substs)

        first_path = desc.evaluate(lc)
        assert desc.allow_substs is (first == 'true')
        assert (os.fspath(first_path) != file_name) is (first == 'true')
        desc.cleanup()
        assert os.path.exists(file_name)

        allow_substs.set_text(second)
        second_path = desc.evaluate(lc)
        assert desc.allow_substs is (second == 'true')
        assert (os.fspath(second_path) != file_name) is (second == 'true')
        desc.cleanup()
        assert os.path.exists(file_name)
        assert isinstance(desc.allow_substs, list)
