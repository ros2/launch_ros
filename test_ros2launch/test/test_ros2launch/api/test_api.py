# Copyright 2020 Canonical, Ltd.
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

"""Tests for the launch api."""

import pytest

from ros2launch.api.api import _Keystore
from ros2launch.api.api import InvalidKeystoreError
from ros2launch.api.api import NoKeystoreProvidedError
from ros2launch.api.api import NonexistantKeystoreError

import sros2.keystore


@pytest.fixture
def keystore_path(tmp_path):
    sros2.keystore.create_keystore(tmp_path)
    return tmp_path


def test__keystore_existing_keystore(keystore_path):
    res = _Keystore(keystore_path=keystore_path, create_keystore=False)
    assert sros2.keystore.is_valid_keystore(keystore_path)

    # Test that it doesn't delete a provided keystore when done
    del res
    assert sros2.keystore.is_valid_keystore(keystore_path)


def test__keystore_valid_path_uninitialized_success(tmp_path):
    res = _Keystore(keystore_path=tmp_path, create_keystore=True)
    assert sros2.keystore.is_valid_keystore(tmp_path)

    del res
    assert sros2.keystore.is_valid_keystore(tmp_path)


def test__keystore_valid_path_uninitialized_fail(tmp_path):
    with pytest.raises(InvalidKeystoreError):
        _Keystore(keystore_path=tmp_path, create_keystore=False)


def test__keystore_valid_path_nonexistant_success(tmp_path):
    res = _Keystore(keystore_path=tmp_path / 'foo', create_keystore=True)
    assert sros2.keystore.is_valid_keystore(tmp_path / 'foo')

    del res
    assert sros2.keystore.is_valid_keystore(tmp_path / 'foo')


def test__keystore_valid_path_nonexistant_fail(tmp_path):
    with pytest.raises(NonexistantKeystoreError):
        _Keystore(keystore_path=tmp_path / 'foo', create_keystore=False)


def test__keystore_transient_success():
    res = _Keystore(keystore_path=None, create_keystore=True)
    assert sros2.keystore.is_valid_keystore(res.path)

    # Test than keystore is cleaned up correctly
    res = res.path
    assert not res.exists()


def test__keystore_transient_failure():
    with pytest.raises(NoKeystoreProvidedError):
        _Keystore(keystore_path=None, create_keystore=False)
