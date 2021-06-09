# Copyright 2021 Apex.AI, Inc.
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


"""Tests for the RosTimerAction Action."""
import sys

import launch
import launch.event_handlers

from launch_ros.actions import Node
from launch_ros.actions import RosTimerAction
from launch_ros.actions import SetParameter
from launch_ros.actions import UseSimTimeAction

import time

def test_multiple_launch_with_timers():
    def generate_launch_description():
        return launch.LaunchDescription([

            launch.actions.ExecuteProcess(
                cmd=[sys.executable, '-c', 'while True: pass'],
            ),

            RosTimerAction(
                period=1.,
                actions=[
                    launch.actions.Shutdown(reason='Timer expired')
                ]
            ),
        ])

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()


def _shutdown_listener_factory(reasons_arr):
    return launch.actions.RegisterEventHandler(
        launch.event_handlers.OnShutdown(
            on_shutdown=lambda event, context: reasons_arr.append(event)
        )
    )


def test_timer_action_sanity_check():
    """Test that timer actions work (sanity check)."""
    # This test is structured like test_shutdown_preempts_timers and
    # test_timer_can_block_preemption as a sanity check that the shutdown listener
    # and other launch related infrastructure works as expected
    shutdown_reasons = []

    ld = launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        RosTimerAction(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='One second timeout')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert shutdown_reasons[0].reason == 'One second timeout'


def test_shutdown_preempts_timers():
    shutdown_reasons = []

    ld = launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        RosTimerAction(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='fast shutdown')
            ]
        ),

        RosTimerAction(
            period=2.,
            actions=[
                launch.actions.Shutdown(reason='slow shutdown')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 1
    assert shutdown_reasons[0].reason == 'fast shutdown'


def test_time_is_passing():
    shutdown_reasons = []

    ld = launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        RosTimerAction(
            period=5.,
            actions=[
                launch.actions.Shutdown(reason='Five second timeout')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    # More than 5 seconds should pass between start of test and shutdown
    start_time = time.time()
    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert abs(time.time() - start_time) > 5
    assert shutdown_reasons[0].reason == 'Five second timeout'


def test_timer_uses_sim_time():
    """Test that timer uses time from /clock topic."""
    ld = launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        Node(
            package='test_launch_ros',
            executable='clock_publisher',
            name='clock_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
                'rate': 100
            }]
        ),

        #SetParameter(name='use_sim_time', value=True),
        UseSimTimeAction(True),

        RosTimerAction(
            period=200.0,
            actions=[
                launch.actions.Shutdown(reason='timer expired')
            ]
        ),
    ])

    start_time = time.time()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    # Simulated time is 100x faster, so 200 sec timer should finish in 2 sec
    assert (time.time() - start_time < 4)
