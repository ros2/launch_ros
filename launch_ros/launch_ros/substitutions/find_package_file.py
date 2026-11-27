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

"""Module for the FindPackageFile substitution."""

import os

from typing import Dict
from typing import List
from typing import Sequence
from typing import Text
from typing import Tuple
from typing import Type

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from .find_package import FindPackageShare


@expose_substitution('find-pkg-file')
class FindPackageFile(FindPackageShare):
    """
    Substitution that locates a file relative to the share directory of a ROS package.

    The share directory is located using ``ament_index_python`` and the given
    relative path is resolved against it. An error is raised during substitution
    if the resolved path does not exist, which catches typos and missing
    install rules early instead of failing downstream.

    For example, the following will return the path of the
    ``example.launch.py`` file in the ``launch`` directory of the
    ``launch_tutorial`` package share directory:

    .. tabs::

        .. tab:: XML
            .. code-block:: xml

                <let name="launch_file"
                     value="$(find-pkg-file launch_tutorial launch/example.launch.py)"/>

        .. tab:: YAML
            .. code-block:: yaml

                launch:
                    - let:
                        name: launch_file
                        value: $(find-pkg-file launch_tutorial launch/example.launch.py)

        .. tab:: Python
            .. code-block:: python

                FindPackageFile('launch_tutorial', 'launch/example.launch.py')

    :raise: ament_index_python.packages.PackageNotFoundError when package is
        not found during substitution
    :raise: SubstitutionFailure when the file is an absolute path, or is not
        found under the package share directory, during substitution
    """

    def __init__(
        self,
        package: SomeSubstitutionsType,
        file: SomeSubstitutionsType,
    ) -> None:
        """Create a FindPackageFile substitution."""
        super().__init__(package)
        self.__file = normalize_to_list_of_substitutions(file)

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type['FindPackageFile'], Dict[str, SomeSubstitutionsType]]:
        """Parse a FindPackageFile substitution."""
        if not data or len(data) != 2:
            raise AttributeError('find package file substitution expects 2 arguments')
        kwargs = {'package': data[0], 'file': data[1]}
        return cls, kwargs

    @property
    def file(self) -> List[Substitution]:
        """Getter for file."""
        return self.__file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        pkg_str = ' + '.join([sub.describe() for sub in self.package])
        file_str = ' + '.join([sub.describe() for sub in self.file])
        return 'FindPackageFile(pkg={}, file={})'.format(pkg_str, file_str)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the file in the package share directory."""
        share_directory = super().perform(context)
        package = perform_substitutions(context, self.package)
        file = perform_substitutions(context, self.file)
        if os.path.isabs(file):
            raise SubstitutionFailure(
                'expected a path relative to the share directory of package {!r}, '
                'got absolute path {!r}'.format(package, file))
        result = os.path.join(share_directory, file)
        if not os.path.exists(result):
            raise SubstitutionFailure(
                'file {!r} not found under the share directory {!r} of package '
                '{!r}'.format(file, share_directory, package))
        return result
