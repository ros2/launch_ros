# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Tests for conditional if/unless on param elements."""

import io
import textwrap

import pytest

from launch import LaunchService
from launch.frontend import Parser

from launch_ros.utilities import evaluate_parameters


def _get_evaluated_params(xml_file):
    """Parse XML, run launch service, and return evaluated parameters."""
    with io.StringIO(xml_file) as f:
        root_entity, parser = Parser.load(f)
        ld = parser.parse_description(root_entity)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 == ls.run()
        evaluated = evaluate_parameters(
            ls.context, ld.describe_sub_entities()[1]._Node__parameters)
        all_keys = {}
        for p in evaluated:
            if isinstance(p, dict):
                all_keys.update(p)
        return all_keys


def test_param_if_true():
    """Param with if=true should be included."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="flag" value="true"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="included" value="1" if="$(var flag)"/>
                <param name="always" value="2"/>
            </node>
        </launch>
        """)
    keys = _get_evaluated_params(xml_file)
    assert 'included' in keys
    assert 'always' in keys


def test_param_if_false():
    """Param with if=false should be excluded."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="flag" value="false"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="excluded" value="1" if="$(var flag)"/>
                <param name="always" value="2"/>
            </node>
        </launch>
        """)
    keys = _get_evaluated_params(xml_file)
    assert 'excluded' not in keys
    assert 'always' in keys


def test_param_unless_true():
    """Param with unless=true should be excluded."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="flag" value="true"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="excluded" value="1" unless="$(var flag)"/>
                <param name="always" value="2"/>
            </node>
        </launch>
        """)
    keys = _get_evaluated_params(xml_file)
    assert 'excluded' not in keys
    assert 'always' in keys


def test_param_unless_false():
    """Param with unless=false should be included."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="flag" value="false"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="included" value="1" unless="$(var flag)"/>
                <param name="always" value="2"/>
            </node>
        </launch>
        """)
    keys = _get_evaluated_params(xml_file)
    assert 'included' in keys
    assert 'always' in keys


def test_param_if_and_unless_raises():
    """Using both if and unless on same param should raise RuntimeError."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="flag" value="true"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="bad" value="1" if="$(var flag)" unless="$(var flag)"/>
            </node>
        </launch>
        """)
    with pytest.raises(RuntimeError, match="if and unless"):
        with io.StringIO(xml_file) as f:
            root_entity, parser = Parser.load(f)
            parser.parse_description(root_entity)


def test_param_no_condition():
    """Param without if/unless should always be included (regression test)."""
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="unused" value="x"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_node">
                <param name="param1" value="hello"/>
                <param name="param2" value="world"/>
            </node>
        </launch>
        """)
    keys = _get_evaluated_params(xml_file)
    assert 'param1' in keys
    assert 'param2' in keys
