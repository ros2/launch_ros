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

"""Module with YamlLikeSubstitution."""

from launch import LaunchContext
from launch import Substitution


class YamlLikeSubstitution(Substitution):
    """
    Implementation detail for `evaluate_parameters`, `normalize_parameters`, do not use it.

    `TextSubstitution` is not evaluated like yaml in `evaluate_paramters`, this one yes.
    """

    def __init__(self, *, text) -> None:
        """Constructor."""
        super().__init__()
        self.__text = text

    def describe(self):
        """Return description."""
        return "'{}'".format(self.__text)

    def perform(self, context: LaunchContext):
        """Perform substitution."""
        return self.__text
