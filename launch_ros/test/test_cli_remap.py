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
import unittest
import time


class TestRemapArgument(unittest.TestCase):
    """Test the --remap command line argument for ros2 launch."""

    def test_remap_argument(self):
        """Test that the --remap argument correctly remaps topics."""
        launch_proc_remapped = None
        try:
            # Start the talker_listener launch file with remapping
            launch_proc_remapped = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "demo_nodes_cpp",
                    "talker_listener_launch.py",
                    "--remap",
                    "/chatter:=/chatter_remapped",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            
            # Give the nodes time to start and register their topics
            time.sleep(5)
            
            # Run ros2 topic list to get the remapped topics
            # Using run without check=True to handle errors
            result = subprocess.run(
                ["ros2", "topic", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            
            # Check if the command was successful
            if result.returncode != 0:
                self.fail(f"'ros2 topic list' command failed with error: {result.stderr}")
                
            remapped_topics = result.stdout.strip().split("\n")
            
            # Verify /chatter is NOT in the list
            self.assertNotIn(
                "/chatter",
                remapped_topics,
                f"Did not expect to find /chatter after remapping. Topics: {remapped_topics}",
            )
            
            # Verify /chatter_remapped IS in the list
            self.assertIn(
                "/chatter_remapped",
                remapped_topics,
                f"Expected to find /chatter_remapped after remapping. Topics: {remapped_topics}",
            )
        finally:
            # Clean up
            if launch_proc_remapped is not None:
                launch_proc_remapped.terminate()
                try:
                    launch_proc_remapped.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    launch_proc_remapped.kill()
                    launch_proc_remapped.wait()


if __name__ == "__main__":
    unittest.main()
    
