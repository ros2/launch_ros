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

from ros2cli.plugin_system import instantiate_extensions
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class OptionExtension:
    """
    The extension point for ros2 launch 'option' extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods may be defined:
    * `add_arguments` to add arguments to the argparse parser
    * `prestart` to perform any actions the extension wishes performed prior
      to the LaunchService being started.
      This method does not need to return anything.
    * `prelaunch` to perform any actions the extension wishes performed prior
      to the launch process running (but after creation and setup).
      This method must return a tuple where the first element is a
      LaunchDescription, which will replace the LaunchDescription that was
      passed in and will be launched by the launch service.
    * `postlaunch` to perform any cleanup actions the extension wishes
      performed after the launch process finishes.
      This method does not need to return anything.
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(OptionExtension, self).__init__()
        satisfies_version(self.EXTENSION_POINT_VERSION , '^0.1')

    def add_arguments(self, parser, cli_name, *, argv=None):
        return

    def prestart(self, options, args):
        return

    def prelaunch(self, launch_description, options, args):
        return (launch_description,)

    def postlaunch(self, launch_return_code, options, args):
        return


def get_option_extensions(exclude_names=None):
    extensions = instantiate_extensions(
        'ros2launch.option',
        exclude_names=exclude_names)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions
