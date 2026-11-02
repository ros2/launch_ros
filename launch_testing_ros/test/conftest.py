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

import faulthandler
import os

import psutil
import pytest

# pytest-timeout (see pytest.ini) bounds each test, but nothing bounds collection,
# the gaps between tests, or interpreter shutdown after the session ends (e.g. a
# non-daemon thread or atexit handler that never returns). A faulthandler watchdog
# covers those phases: on a hang it dumps every thread's stack and exits, rather
# than stalling until the CI job's global timeout.
WATCHDOG_TIMEOUT = 300
SHUTDOWN_WATCHDOG_TIMEOUT = 120
LEAKED_PROCESS_TERMINATE_TIMEOUT = 5

_watchdog_fd = None


def _arm_watchdog(timeout):
    faulthandler.dump_traceback_later(timeout, exit=True, file=_watchdog_fd)


def _kill_leaked_processes(terminalreporter):
    # Processes left behind by launch tests can keep the interpreter from exiting,
    # or hold the output pipe open so colcon never sees the test run finish.
    children = psutil.Process().children(recursive=True)
    if not children:
        return
    for child in children:
        try:
            description = ' '.join(child.cmdline()) or child.name()
        except psutil.Error:
            description = '<unknown>'
        terminalreporter.write_line(
            f'Terminating leaked process {child.pid}: {description}', yellow=True)
        try:
            child.terminate()
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(children, timeout=LEAKED_PROCESS_TERMINATE_TIMEOUT)
    for child in alive:
        terminalreporter.write_line(f'Killing leaked process {child.pid}', yellow=True)
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass


def pytest_configure(config):
    global _watchdog_fd
    # Duplicate stderr while pytest's output capture is suspended, so the stack
    # dump reaches the real stderr no matter when the watchdog fires.
    _watchdog_fd = os.dup(2)
    _arm_watchdog(WATCHDOG_TIMEOUT)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_protocol(item, nextitem):
    # pytest-timeout is responsible for the test itself.
    faulthandler.cancel_dump_traceback_later()
    yield
    _arm_watchdog(WATCHDOG_TIMEOUT)


def pytest_sessionfinish(session, exitstatus):
    terminalreporter = session.config.pluginmanager.get_plugin('terminalreporter')
    _kill_leaked_processes(terminalreporter)


@pytest.hookimpl(trylast=True)
def pytest_unconfigure(config):
    # Left armed through interpreter finalization (non-daemon thread joins, atexit).
    _arm_watchdog(SHUTDOWN_WATCHDOG_TIMEOUT)
