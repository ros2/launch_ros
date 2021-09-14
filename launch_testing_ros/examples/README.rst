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

``launch_test check_msgs_launch_test.py``

The output should be similar to:
::
   [INFO] [launch]: All log files can be found below /home/aditya/.ros/log/2021-09-14-11-12-10-734588-aditya-desktop-2020689
   [INFO] [launch]: Default logging verbosity is set to INFO
   test_check_if_msgs_published (check_msgs_launch_test.TestFixture) ... 1631643130.800377 [0] launch_tes: using network interface enp0s25 (udp/172.23.2.248) selected arbitrarily from: enp0s25, virbr0
   [INFO] [talker-1]: process started with pid [2020700]
   [talker-1] 1631643130.834719 [0]     talker: using network interface enp0s25 (udp/172.23.2.248) selected arbitrarily from: enp0s25, virbr0
   [talker-1] [INFO] [1631643131.848249515] [demo_node_1]: Publishing: 'Hello World: 1'
   ok
   
   ----------------------------------------------------------------------
   Ran 1 test in 1.062s
   
   OK
   [INFO] [talker-1]: sending signal 'SIGINT' to process[talker-1]
   [talker-1] [INFO] [1631643131.855748369] [rclcpp]: signal_handler(signal_value=2)
   [INFO] [talker-1]: process has finished cleanly [pid 2020700]
   
   ----------------------------------------------------------------------
   Ran 0 tests in 0.000s
   
   OK

Generic examples
----------------

More ``launch_testing`` examples which are not ROS specific are available `here <https://github.com/ros2/launch/tree/master/launch_testing/test/launch_testing/examples>`__ with their explanation available `here <https://github.com/ros2/launch/tree/master/launch_testing#examples>`__
