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

"""Test configuration for test_launch_ros."""

import os
import sys

# Opening a shared memory segment left inconsistent by an abruptly terminated
# peer faults inside Fast DDS. See
# https://github.com/ros2/launch_ros/issues/539
if sys.platform == 'win32':
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
