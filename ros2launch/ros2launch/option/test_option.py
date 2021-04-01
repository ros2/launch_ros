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

from ros2cli.command import add_subparsers_on_demand
from ros2launch.option import OptionExtension


class TestOption(OptionExtension):
    """Test adding an extension option to ros2 launch."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-t',
            '--test-option',
            action='store_true',
            help='Test extensible options'
        )

    def prestart(self, args):
        if args.test_option:
            print('Performing pre-start actions')

    def prelaunch(self, launch_description, args):
        if args.test_option:
            print('Performing pre-launch actions')
        return (launch_description,)

    def postlaunch(self, launch_return_code, args):
        if args.test_option:
            print(f'Performing post-launch actions, return code: {launch_return_code}')
