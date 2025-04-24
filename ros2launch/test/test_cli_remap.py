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

import logging
import os
import signal
import sys
import threading
import time
import unittest

from ament_index_python.packages import PackageNotFoundError
import rclpy
from rclpy.node import Node

from ros2launch.api.api import (
    get_share_file_path_from_package,
    launch_a_launch_file,
    MultipleLaunchFilesError
)

# Configure basic logging
logging.basicConfig(level=logging.INFO, stream=sys.stderr,
                    format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s')


class TestRemapArgument(unittest.TestCase):
    """Test the --remap command line argument for ros2 launch."""

    def test_remap_argument(self):
        """Test that the --remap argument correctly remaps topics using direct API call."""
        logging.info('Starting test_remap_argument...')

        checker_thread = None
        # Flag to indicate test success from the checker thread
        test_successful = threading.Event()
        test_failed_assertion = None

        def check_topics_target():
            nonlocal test_failed_assertion
            logging.info('Checker thread started.')

            # Initialize rclpy and create node for checking topics
            logging.info('Checker thread: Initializing rclpy...')
            # Ensure rclpy initializes in this thread context if necessary
            # It might inherit context or need specific args depending on ROS setup
            try:
                rclpy.init()
            except Exception as e:
                logging.error(f'Checker thread: rclpy.init() failed: {e}', exc_info=True)
                # Can't proceed without rclpy
                os.kill(os.getpid(), signal.SIGINT)  # Signal main thread to stop
                return

            node = None
            try:
                logging.info('Checker thread: Creating node __test_remap_checker...')
                node = Node('__test_remap_checker', use_global_arguments=False)
                logging.info('Checker thread: Node created.')

                # Wait for the remapped topic to appear, poll for it
                start_time = time.time()
                timeout = 25.0
                remapped_topic_found = False
                iteration = 0

                logging.info('Checker thread: Starting topic polling loop...')
                while time.time() - start_time < timeout:
                    iteration += 1
                    # Check if main thread is still alive (optional, sanity check)

                    topic_names_and_types = node.get_topic_names_and_types()
                    current_topics = [name for name, types in topic_names_and_types]
                    logging.info(f'Poll {iteration}. Topic count: {len(current_topics)}')

                    if '/chatter_remapped' in current_topics:
                        logging.info("Checker thread: Found target topic '/chatter_remapped'!")
                        remapped_topic_found = True
                        break

                    time.sleep(0.5)

                log_msg = f'Polling finished. Found remapped topic: {remapped_topic_found}'
                logging.info(f'Checker thread: {log_msg}')

                final_topic_names_and_types = node.get_topic_names_and_types()
                final_topics = [name for name, types in final_topic_names_and_types]
                logging.info(f'Checker thread: Final topics: {final_topics}')

                # Perform assertions within this thread using self.assertX methods
                try:
                    logging.info('Checker thread: Asserting /chatter_remapped is present...')
                    msg = (
                        f'Expected topic "/chatter_remapped" not found within {timeout}s. '
                        f'Final topics: {final_topics}'
                    )
                    self.assertTrue(remapped_topic_found, msg)
                    logging.info('Checker thread: Assertion passed.')

                    logging.info('Checker thread: Asserting /chatter is NOT present...')
                    msg = (
                        f'Unexpectedly found original topic "/chatter". '
                        f'Final topics: {final_topics}'
                    )
                    self.assertNotIn('/chatter', final_topics, msg)
                    logging.info('Checker thread: Assertion passed.')

                    # If assertions pass, set the success flag
                    test_successful.set()
                except AssertionError as e:
                    logging.error(f'Checker thread: Assertion failed: {e}')
                    test_failed_assertion = e  # Store assertion for main thread reporting
            except Exception as e:
                # Catch and print any other exceptions from the launch thread
                logging.error(f'Checker thread: Error during checks: {e}', exc_info=True)
            finally:
                # Clean up node and rclpy
                if node is not None:
                    logging.info('Checker thread: Destroying node...')
                    node.destroy_node()
                if rclpy.ok():
                    logging.info('Checker thread: Shutting down rclpy...')
                    rclpy.shutdown()
                logging.info('Checker thread: rclpy cleanup finished.')

                # Signal the main thread (running launch) to stop
                log_msg = 'Signaling main thread (SIGINT) to stop launch service...'
                logging.info(f'Checker thread: {log_msg}')
                os.kill(os.getpid(), signal.SIGINT)
                logging.info('Checker thread: Exiting.')

        # Main thread execution starts here
        try:
            logging.info('Main thread: Creating checker thread...')
            # Make checker non-daemon so main thread waits for it via join()
            checker_thread = threading.Thread(target=check_topics_target, daemon=False)
            logging.info('Main thread: Starting checker thread...')
            checker_thread.start()

            launch_return_code = None
            try:
                # Find launch file (moved here from background thread)
                logging.info('Main thread: Finding launch file...')
                package = 'demo_nodes_cpp'
                file = 'talker_listener_launch.py'
                launch_file_path = get_share_file_path_from_package(
                    package_name=package, file_name=file
                )
                logging.info(f'Main thread: Found launch file: {launch_file_path}')

                remap_rules = ['/chatter:=/chatter_remapped']

                # Call launch_a_launch_file in the main thread
                logging.info('Main thread: Calling launch_a_launch_file (blocking)...')
                launch_return_code = launch_a_launch_file(
                    launch_file_path=launch_file_path,
                    launch_file_arguments=[],
                    noninteractive=True,
                    debug=False,
                    remap_rules=remap_rules
                )
                log_msg = f'launch_a_launch_file returned with code: {launch_return_code}'
                logging.info(f'Main thread: {log_msg}')

            except (PackageNotFoundError, FileNotFoundError, MultipleLaunchFilesError) as e:
                logging.error(f'Main thread: Error finding launch file: {e}')
                # Ensure checker thread is stopped if launch setup failed
                if checker_thread and checker_thread.is_alive():
                    log_msg = 'Signaling checker thread to stop due to launch error...'
                    logging.info(f'Main thread: {log_msg}')
                    # Ideally have a cleaner way, but SIGINT might work if thread handles it
                    # Or rely on join timeout below
            except Exception as e:
                logging.error(f'Main thread: Error during launch: {e}', exc_info=True)
            finally:
                # Wait for the checker thread to finish its checks and signal
                logging.info('Main thread: Joining checker thread...')
                if checker_thread is not None:
                    checker_thread.join(timeout=30.0)  # Increased timeout to allow checks
                    if checker_thread.is_alive():
                        logging.warning('Checker thread is still alive after timeout.')
                    else:
                        logging.info('Checker thread joined successfully.')

        # Outer try block needs a finally or except
        # Re-adding the final check logic here which belongs to the outer try
        finally:
            # After launch and checker thread have finished, check assertion results
            logging.info('Main thread: Checking test results...')
            if test_failed_assertion:
                # Re-raise the assertion failure captured from the checker thread
                raise test_failed_assertion
            elif not test_successful.is_set():
                # Fail if the checker thread didn't explicitly signal success
                msg = 'Test failed: Checker thread did not signal success'
                self.fail(msg)
            else:
                logging.info('Main thread: Test success confirmed by checker thread.')

            logging.info('Main thread: Test finished.')


if __name__ == '__main__':
    unittest.main() 