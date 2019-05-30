^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2019-05-29)
------------------
* Added the ``FindPackage`` substitution. (`#22 <https://github.com/ros2/launch_ros/issues/22>`_)
* Changed interpretation of Parameter values which are passed to ``Node()`` so that they get evaluated by yaml rules. (`#31 <https://github.com/ros2/launch_ros/issues/31>`_)
* Contributors: Shane Loretz, ivanpauno

0.8.2 (2019-05-20)
------------------
* Fix deprecation warnings (`#25 <https://github.com/ros2/launch_ros/issues/25>`_)
* Corrected matches_action usage in lifecycle_pub_sub example (`#21 <https://github.com/ros2/launch_ros/issues/21>`_)
* Contributors: Jacob Perron, ivanpauno

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-14)
------------------
* Make 'ros2 launch' work again. (`#201 <https://github.com/ros2/launch_ros/issues/201>`_)
* Added LaunchLogger class (`#145 <https://github.com/ros2/launch/issues/145>`_)
* Changed logger.warn (deprecated) to logger.warning. (`#199 <https://github.com/ros2/launch/issues/199>`_)
* Added Plumb rclpy.init context to get_default_launch_description. (`#193 <https://github.com/ros2/launch/issues/193>`_)
* Added normalize_parameters and evaluate_paramters. (`#192 <https://github.com/ros2/launch/issues/192>`_)
* Added normalize_remap_rule and types. (`#173 <https://github.com/ros2/launch/issues/173>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Jacob Perron, Peter Baughman, Shane Loretz

0.7.3 (2018-12-13)
------------------

0.7.2 (2018-12-06)
------------------

0.7.1 (2018-11-16)
------------------
* Fixed setup.py versions (`#155 <https://github.com/ros2/launch/issues/155>`_)
* Contributors: Steven! Ragnarök

0.7.0 (2018-11-16)
------------------
* Renamed transitions to match changes in ``lifecycle_msgs`` (`#153 <https://github.com/ros2/launch/issues/153>`_)
  * TRANSITION_SHUTDOWN was deleted in ``lifecycle_msgs/msg/Transition.msg``
  * Align with the code changes from https://github.com/ros2/rcl_interfaces/commit/852a37ba3ae0f7e58f4314fa432a8ea7f0cbf639
  * Signed-off-by: Chris Ye <chris.ye@intel.com>
* Added 'handle_once' property for unregistering an EventHandler after one event (`#141 <https://github.com/ros2/launch/issues/141>`_)
* Added support for passing parameters as a dictionary to a Node (`#138 <https://github.com/ros2/launch/issues/138>`_)
* Made various fixes and added tests for remappings passed to Node actions (`#137 <https://github.com/ros2/launch/issues/137>`_)
* Added ability to pass parameter files to Node actions (`#135 <https://github.com/ros2/launch/issues/135>`_)
* Added ability to define and pass launch arguments to launch files (`#123 <https://github.com/ros2/launch/issues/123>`_)
  * See changelog in ``launch`` for details.
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: Chris Ye, Jacob Perron, William Woodall, dhood

0.6.0 (2018-08-20)
------------------
* Fixed a bug where launch would hang on exit by destroying the rclpy node on shutdown (`#124 <https://github.com/ros2/launch/issues/124>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Fixed a race condition in emitting events by using loop.call_soon_threadsafe() (`#119 <https://github.com/ros2/launch/issues/119>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: William Woodall

0.5.2 (2018-07-17)
------------------

0.5.1 (2018-06-27)
------------------
* Various Windows fixes. (`#87 <https://github.com/ros2/launch/issues/87>`_)
* Contributors: William Woodall

0.5.0 (2018-06-19)
------------------
* Changed to use variable typing in comments to support python 3.5 (`#81 <https://github.com/ros2/launch/issues/81>`_)
* First commit of the ROS specific launch API (`#75 <https://github.com/ros2/launch/issues/75>`_)
  * ROS specific functionality for the new launch API.
* Contributors: William Woodall, dhood
