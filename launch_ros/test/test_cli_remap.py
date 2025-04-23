#!/usr/bin/env python3
# Copyright 2025 Open Source Robotics Foundation, Inc.
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
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node


class TestRemapArgument(unittest.TestCase):
    """Test the --remap command line argument for ros2 launch."""

    def test_remap_argument(self):
        """Test that the --remap argument correctly remaps topics."""
        launch_proc_remapped = None
        try:
            # Start the talker_listener launch file with remapping
            # Note: Using shell=True to allow sourcing the setup file.
            # Ensure ROS_DISTRO is set in the environment where this test runs.
            command_str = (
                'source /opt/ros/$ROS_DISTRO/setup.bash && '
                'ros2 launch demo_nodes_cpp talker_listener_launch.py '
                '--remap /chatter:=/chatter_remapped'
            )
            launch_proc_remapped = subprocess.Popen(
                command_str,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,  # Execute through the shell
                executable='/bin/bash'  # Be explicit about the shell
            )

            # Initialize rclpy and create node for checking topics
            stdout_log = ''
            stderr_log = ''
            rclpy.init()
            node = None
            try:
                node = Node('__test_remap_checker', use_global_arguments=False)

                # Wait for the remapped topic to appear, poll for it
                start_time = time.time()
                timeout = 15.0  # seconds
                remapped_topic_found = False

                while time.time() - start_time < timeout:
                    topic_names_and_types = node.get_topic_names_and_types()
                    current_topics = [name for name,
                                      types in topic_names_and_types]

                    if '/chatter_remapped' in current_topics:
                        remapped_topic_found = True
                        break  # Found the target topic

                    # Avoid busy-waiting
                    time.sleep(0.5)

                # Get the final list of topics after the loop/timeout
                final_topic_names_and_types = node.get_topic_names_and_types()
                final_topics = [name for name,
                                types in final_topic_names_and_types]

                # Capture stdout and stderr before asserting
                if launch_proc_remapped and not remapped_topic_found:
                    # Ensure the process is terminated before reading streams
                    if launch_proc_remapped.poll() is None:
                        launch_proc_remapped.terminate()
                        try:
                            launch_proc_remapped.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            if launch_proc_remapped.poll() is None:
                                launch_proc_remapped.kill()
                                launch_proc_remapped.wait(timeout=1)
                    # Now read the streams
                    stdout_bytes, stderr_bytes = launch_proc_remapped.communicate()
                    stdout_log = stdout_bytes.decode('utf-8', errors='replace')
                    stderr_log = stderr_bytes.decode('utf-8', errors='replace')

                # Verify /chatter_remapped IS in the list
                self.assertTrue(
                    remapped_topic_found,
                    f'Expected topic "/chatter_remapped" not found within {timeout}s.\n'
                    f'Final topics found: {final_topics}\n'
                    f'--- Launch STDOUT ---\n{stdout_log}\n'
                    f'--- Launch STDERR ---\n{stderr_log}'
                )

                # Verify /chatter is NOT in the list
                self.assertNotIn(
                    '/chatter',
                    final_topics,
                    f'Unexpectedly found original topic "/chatter". Final topics: {
                        final_topics}'
                )

            finally:
                # Clean up node and rclpy
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()

        finally:
            # Clean up the launch process
            if launch_proc_remapped is not None and launch_proc_remapped.poll() is None:
                # Ensure process is terminated if not already done for logging
                launch_proc_remapped.terminate()
                try:
                    launch_proc_remapped.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if launch_proc_remapped.poll() is None:  # Check again if terminate failed
                        launch_proc_remapped.kill()
                        launch_proc_remapped.wait()


if __name__ == '__main__':
    unittest.main()
