Launch testing examples
=======================

.. contents:: Contents
   :depth: 2
   :local:

Overview
--------

This directory contains examples to help the user get an idea of how to use ``launch_testing`` to write their own test cases. It contains the following examples :

* ``check_node_launch_test.py`` : There might be situations where nodes, once launched, take some time to actually start and we need to wait for the node to start to perform some action. We can simulate this using ``launch.actions.TimerAction``. This example shows one way to detect when a node has been launched. We delay the launch by 5 seconds, and wait for the node to start with a timeout of 8 seconds.
* ``check_msgs_launch_test.py`` : Consider a problem statement where you need to launch a node and check if messages are published on a particular topic. This example demonstrates how to do that, using the talker node from ``demo_nodes_cpp`` package. It uses the ``Event`` object to end the test as soon as the first message is received on the chatter topic, with a timeout of 5 seconds. It is recommended to keep the tests as short as possible.
* ``set_param_launch_test.py`` : Launch a node, set a parameter in it and check if that was successful.

Running the tests
-----------------
Make sure you have your ROS workspace sourced and then run:

``launch_test hello_world_launch_test.py``

The output should be similar to:
::
  [INFO] [launch]: All log files can be found below /home/aditya/.ros/log/2021-08-23-09-47-06-613035-aditya-desktop-1376669
  [INFO] [launch]: Default logging verbosity is set to INFO
  test_read_stdout (hello_world.TestHelloWorldProcess) ... [INFO] [echo-1]: process started with pid [1376672]
  ok
  
  [INFO] [echo-1]: process has finished cleanly [pid 1376672]
  ----------------------------------------------------------------------
  Ran 1 test in 0.008s
  
  OK
  test_exit_codes (hello_world.TestHelloWorldShutdown) ... ok
  
  ----------------------------------------------------------------------
  Ran 1 test in 0.000s
  
  OK

Similarly, the other tests can be run after making sure the corresponding dependencies are installed.

Generic examples
----------------

More ``launch_testing`` examples which are not ROS specific are available `here <https://github.com/ros2/launch/tree/master/launch_testing/test/launch_testing/examples>`__ with their explanation available `here <https://github.com/ros2/launch/tree/master/launch_testing#examples>`__
