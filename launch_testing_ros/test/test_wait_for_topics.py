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

import gc
import os
import subprocess
import sys

from launch_testing_ros.wait_for_topics import WaitForTopics
import pytest
from std_msgs.msg import String


def test_daemon_thread_and_idempotent_shutdown():
    """Verify that spin thread is a daemon and shutdown is idempotent."""
    w = WaitForTopics([('dummy_topic', String)], timeout=0.1)
    spin_thread = getattr(w, '_WaitForTopics__ros_spin_thread')
    assert spin_thread.daemon is True
    assert spin_thread.is_alive()
    assert w._finalizer.alive is True

    # Calling shutdown() multiple times should succeed cleanly
    w.shutdown()
    assert w._finalizer.alive is False
    assert not spin_thread.is_alive()
    w.shutdown()
    assert w._finalizer.alive is False


def test_finalizer_cleans_up_on_garbage_collection():
    """Verify deleting WaitForTopics without shutdown() stops its thread on GC."""
    w = WaitForTopics([('dummy_topic', String)], timeout=0.1)
    spin_thread = getattr(w, '_WaitForTopics__ros_spin_thread')
    ros_context = getattr(w, '_WaitForTopics__ros_context')
    assert spin_thread.is_alive()
    assert ros_context.ok()

    del w
    gc.collect()

    assert not spin_thread.is_alive()
    assert not ros_context.ok()


def test_non_callable_trigger_raises_type_error():
    """Verify invalid trigger raises TypeError without leaking resources."""
    with pytest.raises(TypeError):
        WaitForTopics([('dummy_topic', String)], trigger='not_callable')


def test_context_manager_failure_cleans_up():
    """Verify context manager cleans up when wait fails."""
    with pytest.raises(RuntimeError):
        with WaitForTopics([('nonexistent_topic', String)], timeout=0.1):
            pass


def test_process_exit_without_explicit_shutdown():
    """Verify that omitting shutdown() does not block process exit."""
    env = os.environ.copy()
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    env['PYTHONPATH'] = pkg_dir + os.pathsep + env.get('PYTHONPATH', '')

    code = (
        'import sys\n'
        'from launch_testing_ros.wait_for_topics import WaitForTopics\n'
        'from std_msgs.msg import String\n'
        'w = WaitForTopics([("nonexistent_topic", String)], timeout=0.1)\n'
        'w.wait()\n'
        '# Intentionally omit w.shutdown()\n'
        'sys.exit(0)\n'
    )
    proc = subprocess.run(
        [sys.executable, '-c', code],
        timeout=10,
        capture_output=True,
        text=True,
        env=env
    )
    assert proc.returncode == 0, f'Subprocess failed or hung:\n{proc.stderr}'
