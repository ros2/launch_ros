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
import time
import threading
from functools import partial

import launch
import launch.event_handlers

from launch_ros.actions import RosTimerAction

import rclpy
from rclpy.clock import ClockType

import rosgraph_msgs.msg
import builtin_interfaces.msg

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

    # Create clock publisher node
    rclpy.init()
    node = rclpy.create_node('clock_publisher_node')
    publisher = node.create_publisher(rosgraph_msgs.msg.Clock, '/clock', 10)

    # Increment sim time by 100 every second
    def timer_callback(publisher, time_msg):
        time_msg.sec += 100
        publisher.publish(rosgraph_msgs.msg.Clock(clock=time_msg))

    # For every second of system time, publish new sim time value
    callback_clock = rclpy.clock.Clock(clock_type=ClockType.SYSTEM_TIME)
    time_msg = builtin_interfaces.msg.Time(sec=0, nanosec=0)
    node.create_timer(1, partial(timer_callback, publisher, time_msg), clock=callback_clock)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    
    ld = launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        RosTimerAction(
            period=200.0,
            actions=[
                launch.actions.Shutdown(reason='timer expired')
            ],
            use_sim_time = True # must be set to allow timer action to use sim time
        ),
    ])

    start_time = time.time()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    # Simulated time is 100x faster, so 200 sec timer should finish in 2 sec
    assert (time.time() - start_time < 6)
