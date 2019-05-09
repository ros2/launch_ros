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

"""Module for the FindPackage substitution."""

from typing import List
from typing import Text

from ament_index_python.packages import get_package_prefix

from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


class FindPackage(Substitution):
    """
    Substitution that tries to locate the package prefix of a ROS package.

    The ROS package is located using ament_index_python.

    :raise: ament_index_python.packages.PackageNotFoundError when package is
        not found during substitution
    """

    def __init__(self, package: SomeSubstitutionsType) -> None:
        """Constructor."""
        super().__init__()
        self.__package = normalize_to_list_of_substitutions(package)

    @property
    def package(self) -> List[Substitution]:
        """Getter for package."""
        return self.__package

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        pkg_str = ' + '.join([sub.describe() for sub in self.package])
        return 'Pkg(pkg={})'.format(pkg_str)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the executable."""
        package = perform_substitutions(context, self.package)
        result = get_package_prefix(package)
        return result
