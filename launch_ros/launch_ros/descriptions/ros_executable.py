# Copyright 2021 Southwest Research Institute, All Rights Reserved.
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

"""Module for a description of a ROS Executable."""

from typing import Iterable
from typing import Optional

from launch import Action
from launch import LaunchContext
from launch import SomeSubstitutionsType
from launch.descriptions import Executable

from launch_ros.substitutions import ExecutableInPackage

from ..descriptions import Node


class RosExecutable(Executable):
    """Describes an executable with ROS features which may be run by the launch system."""

    def __init__(
            self, *,
            executable: Optional[SomeSubstitutionsType] = None,
            package: Optional[SomeSubstitutionsType] = None,
            nodes: Iterable[Node] = None,
            ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            **kwargs
    ) -> None:
        """
        Initialize an Executable description.

        :param: executable the name of the executable to find if a package
            is provided or otherwise a path to the executable to run.
        :param: package the package in which the node executable can be found
        :param: nodes the ROS node(s) included in the executable
        """
        if package is not None:
            cmd = [ExecutableInPackage(package=package, executable=executable)]
        else:
            cmd = [executable]

        if ros_arguments is not None:
            if arguments is not None:
                arguments.append('--ros-args')
            else:
                arguments = ['--ros-args']
            arguments.extend(ros_arguments)

        self.__package = package
        self.__executable = executable
        self.__nodes = nodes
        super().__init__(cmd=cmd, arguments=arguments, **kwargs)

    @property
    def package(self):
        """Getter for package."""
        return self.__package

    @property
    def executable(self):
        """Getter for executable."""
        return self.__executable

    @property
    def nodes(self):
        """Getter for nodes."""
        return self.__nodes

    def prepare(self, context: LaunchContext, action: Action):
        """
        Prepare a ROS executable description for execution in a given environment.

        This does the following:
        - prepares all nodes
        - performs substitutions on various properties
        """
        for node in self.__nodes:
            node.prepare(context, self, action)

        super().prepare(context, action)
