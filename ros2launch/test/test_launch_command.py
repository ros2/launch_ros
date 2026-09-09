# Copyright 2026 Chuanzong1
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

from argparse import ArgumentParser
from unittest.mock import patch

from ament_index_python.packages import PackageNotFoundError
import pytest

from ros2launch.command.launch import LaunchCommand


def _parse_launch_arguments(arguments):
    parser = ArgumentParser()
    command = LaunchCommand()
    with patch(
        'ros2launch.command.launch.get_option_extensions',
        return_value={},
    ):
        command.add_arguments(parser, 'ros2 launch')
    return command, parser, parser.parse_args(arguments)


@pytest.mark.parametrize(
    'launch_arguments',
    [
        [],
        ['example_arg:=foobar'],
    ],
)
def test_main_launches_file_path(tmp_path, launch_arguments):
    launch_file = tmp_path / 'example.launch.py'
    launch_file.touch()
    command, parser, args = _parse_launch_arguments(
        [str(launch_file), *launch_arguments]
    )

    with patch(
        'ros2launch.command.launch.launch_a_launch_file',
        return_value=0,
    ) as launch_a_launch_file:
        assert command.main(parser=parser, args=args) == 0

    launch_a_launch_file.assert_called_once_with(
        launch_file_path=str(launch_file),
        launch_file_arguments=launch_arguments,
        noninteractive=args.noninteractive,
        args=args,
        option_extensions={},
        debug=False,
    )


@pytest.mark.parametrize(
    'launch_arguments',
    [
        [],
        ['example_arg:=foobar'],
    ],
)
def test_main_launches_package_file(launch_arguments):
    package_name = 'example_package'
    launch_file_name = 'example.launch.py'
    launch_file_path = '/example/share/example_package/example.launch.py'
    command, parser, args = _parse_launch_arguments(
        [package_name, launch_file_name, *launch_arguments]
    )

    with patch(
        'ros2launch.command.launch.get_share_file_path_from_package',
        return_value=launch_file_path,
    ) as get_share_file_path, patch(
        'ros2launch.command.launch.launch_a_launch_file',
        return_value=0,
    ) as launch_a_launch_file:
        assert command.main(parser=parser, args=args) == 0

    get_share_file_path.assert_called_once_with(
        package_name=package_name,
        file_name=launch_file_name,
    )
    launch_a_launch_file.assert_called_once_with(
        launch_file_path=launch_file_path,
        launch_file_arguments=launch_arguments,
        noninteractive=args.noninteractive,
        args=args,
        option_extensions={},
        debug=False,
    )


@pytest.mark.parametrize(
    'exception, expected_message',
    [
        (
            PackageNotFoundError('example_package'),
            "Package 'example_package' not found",
        ),
        (
            FileNotFoundError('example.launch.py not found'),
            'example.launch.py not found',
        ),
    ],
)
def test_main_reports_package_file_errors(exception, expected_message):
    command, parser, args = _parse_launch_arguments(
        ['example_package', 'example.launch.py']
    )

    with patch(
        'ros2launch.command.launch.get_share_file_path_from_package',
        side_effect=exception,
    ), patch(
        'ros2launch.command.launch.launch_a_launch_file',
    ) as launch_a_launch_file:
        with pytest.raises(RuntimeError, match=expected_message):
            command.main(parser=parser, args=args)

    launch_a_launch_file.assert_not_called()
