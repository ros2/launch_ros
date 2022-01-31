^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.6 (2022-01-31)
-------------------
* Validate complex attributes of 'node' action (backport `#198 <https://github.com/ros2/launch_ros/issues/198>`_) (`#293 <https://github.com/ros2/launch_ros/issues/293>`_)
* More Helpful Error Messages (`#275 <https://github.com/ros2/launch_ros/issues/275>`_) (`#289 <https://github.com/ros2/launch_ros/issues/289>`_)
* Contributors: Aditya Pande, David V. Lu!!, Ivan Santiago Paunovic

0.11.5 (2021-11-16)
-------------------
* Fix problems when parsing a ``Command`` ``Substitution`` as a parameter value (`#137 <https://github.com/ros2/launch_ros/issues/137>`_)
* Contributors: Ivan Santiago Paunovic

0.11.4 (2021-10-05)
-------------------

0.11.3 (2021-08-31)
-------------------
* Better document parameter handling in Node (`#234 <https://github.com/ros2/launch_ros/issues/234>`_) (`#242 <https://github.com/ros2/launch_ros/issues/242>`_)
* Fix AttributeError when accessing component container name (`#177 <https://github.com/ros2/launch_ros/issues/177>`_) (`#241 <https://github.com/ros2/launch_ros/issues/241>`_)
* Asynchronously wait for load node service response (`#174 <https://github.com/ros2/launch_ros/issues/174>`_) (`#240 <https://github.com/ros2/launch_ros/issues/240>`_)
* Contributors: Felix Divo, Jacob Perron

0.11.2 (2021-04-14)
-------------------
* Fix namespace with LifecycleNode (`#222 <https://github.com/ros2/launch_ros/issues/222>`_)
* Handle any substitution types for SetParameter name argument (`#182 <https://github.com/ros2/launch_ros/issues/182>`_) (`#218 <https://github.com/ros2/launch_ros/issues/218>`_)
* Contributors: Jacob Perron, Michael Jeronimo

0.11.1 (2020-12-09)
-------------------
* Fix case where list of composable nodes is zero (`#173 <https://github.com/ros2/launch_ros/issues/173>`_) (`#209 <https://github.com/ros2/launch_ros/issues/209>`_)
* Do not use event handler for loading composable nodes (`#170 <https://github.com/ros2/launch_ros/issues/170>`_) (`#208 <https://github.com/ros2/launch_ros/issues/208>`_)
* Fix race with launch context changes when loading composable nodes (`#166 <https://github.com/ros2/launch_ros/issues/166>`_) (`#206 <https://github.com/ros2/launch_ros/issues/206>`_)
* Add a way to set remapping rules for all nodes in the same scope (`#163 <https://github.com/ros2/launch_ros/issues/163>`_) (`#203 <https://github.com/ros2/launch_ros/issues/203>`_)
* Resolve libyaml warning when loading parameters from file (`#161 <https://github.com/ros2/launch_ros/issues/161>`_) (`#202 <https://github.com/ros2/launch_ros/issues/202>`_)
* Fix ComposableNode ignoring PushRosNamespace actions (`#162 <https://github.com/ros2/launch_ros/issues/162>`_) (`#201 <https://github.com/ros2/launch_ros/issues/201>`_)
* Contributors: Dereck Wonnacott, Ivan Santiago Paunovic, Jacob Perron

0.11.0 (2020-10-28)
-------------------
* Update maintainer list for Foxy (`#194 <https://github.com/ros2/launch_ros/issues/194>`_)
* Improve error message if ComposableNodeContainer 'name' or 'namespace' arg is None (`#191 <https://github.com/ros2/launch_ros/issues/191>`_)
* Add a SetParameter action that sets a parameter to all nodes in the same scope (`#158 <https://github.com/ros2/launch_ros/issues/158>`_) (`#187 <https://github.com/ros2/launch_ros/issues/187>`_)
* Avoid using a wildcard to specify parameters if possible (`#154 <https://github.com/ros2/launch_ros/issues/154>`_) (`#188 <https://github.com/ros2/launch_ros/issues/188>`_)
* Assume root namespace if not provided in ComposableNodeContainer (`#186 <https://github.com/ros2/launch_ros/issues/186>`_)
  Fixes a regression introduced in `#179 <https://github.com/ros2/launch_ros/issues/179>`_.
  Fixes `#185 <https://github.com/ros2/launch_ros/issues/185>`_.
  We apply the same logic in LifecycleNode to maintain backwards compatibility in Foxy.
* Contributors: Ivan Santiago Paunovic, Jacob Perron, Michael Jeronimo

0.10.3 (2020-10-07)
-------------------
* Fix no specified namespace (`#153 <https://github.com/ros2/launch_ros/issues/153>`_, `#157 <https://github.com/ros2/launch_ros/issues/157>`_) (`#179 <https://github.com/ros2/launch_ros/issues/179>`_)
* Contributors: Ivan Santiago Paunovic, Jacob Perron

0.10.2 (2020-05-26)
-------------------

0.10.1 (2020-05-13)
-------------------
* Fix new flake8 errors (`#148 <https://github.com/ros2/launch_ros/issues/148>`_)
* Contributors: Michel Hidalgo

0.10.0 (2020-04-29)
-------------------
* Avoid using sys.argv in rclpy.init (`#144 <https://github.com/ros2/launch_ros/issues/144>`_)
* Deprecated 'node_executable' parameter and replace with 'executable' (`#140 <https://github.com/ros2/launch_ros/issues/140>`_)
* Bump node_name warning stacklevel (`#138 <https://github.com/ros2/launch_ros/issues/138>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Enable implicit ROS startup by launch_ros actions  (`#128 <https://github.com/ros2/launch_ros/issues/128>`_)
* Add warning message when launching Non-Uniquely Named Nodes (`#127 <https://github.com/ros2/launch_ros/issues/127>`_)
* Rename node-related parameters (`#122 <https://github.com/ros2/launch_ros/issues/122>`_)
* Fix LoadComposableNodes action so that loading happens asynchronously (`#113 <https://github.com/ros2/launch_ros/issues/113>`_)
* Fix frontend topic remapping (`#111 <https://github.com/ros2/launch_ros/issues/111>`_)
* Check for shutdown while waiting for a service response to avoid hang during shutdown (`#104 <https://github.com/ros2/launch_ros/issues/104>`_)
* Fix misleading deprecated warnings when using launch arguments (`#106 <https://github.com/ros2/launch_ros/issues/106>`_)
* Use imperative mood in constructor docstrings (`#103 <https://github.com/ros2/launch_ros/issues/103>`_)
* Maintain order of parameters regarding name and from (`#99 <https://github.com/ros2/launch_ros/issues/99>`_)
* Allow separate launch composition (`#77 <https://github.com/ros2/launch_ros/issues/77>`_)
* Fix push-ros-namespace in xml/yaml launch files (`#100 <https://github.com/ros2/launch_ros/issues/100>`_)
* Pass the node-name attribute through the substitution parser (`#101 <https://github.com/ros2/launch_ros/issues/101>`_)
* Add pid to launch_ros node name as suffix (`#98 <https://github.com/ros2/launch_ros/issues/98>`_)
* Contributors: Brian Ezequiel Marchi, Brian Marchi, Dirk Thomas, Eric Fang, Grey, Ivan Santiago Paunovic, Jacob Perron, Miaofei Mei, Michel Hidalgo, Shane Loretz, Steven! Ragnarök, William Woodall

0.9.4 (2019-11-19)
------------------
* fix new linter warnings as of flake8-comprehensions 3.1.0 (`#94 <https://github.com/ros2/launch_ros/issues/94>`_)
* Contributors: Dirk Thomas

0.9.3 (2019-11-13)
------------------

0.9.2 (2019-10-23)
------------------
* Fix launch_ros.actions.Node parsing function (`#83 <https://github.com/ros2/launch_ros/issues/83>`_)
* Add support for launching nodes not in a package (`#82 <https://github.com/ros2/launch_ros/issues/82>`_)
* Contributors: Michel Hidalgo

0.9.1 (2019-09-28)
------------------

0.9.0 (2019-09-25)
------------------
* Refactor Node parse() function. (`#73 <https://github.com/ros2/launch_ros/issues/73>`_)
* Handle zero-width string parameters. (`#72 <https://github.com/ros2/launch_ros/issues/72>`_)
* Promote special CLI rules to flags (`#68 <https://github.com/ros2/launch_ros/issues/68>`_)
* Add substitution for finding package share directory (`#57 <https://github.com/ros2/launch_ros/issues/57>`_)
* Use of -r/--remap flags where appropriate. (`#59 <https://github.com/ros2/launch_ros/issues/59>`_)
* install package marker and manifest (`#62 <https://github.com/ros2/launch_ros/issues/62>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#52 <https://github.com/ros2/launch_ros/issues/52>`_)
* Use node namespace if no other was specified (`#51 <https://github.com/ros2/launch_ros/issues/51>`_)
* [launch frontend] Rename some tag attributes (`#47 <https://github.com/ros2/launch_ros/issues/47>`_)
* Fix PushRosNamespace action (`#44 <https://github.com/ros2/launch_ros/issues/44>`_)
* Add PushRosNamespace action (`#42 <https://github.com/ros2/launch_ros/issues/42>`_)
* Add frontend parsing methods for Node, ExecutableInPackage and FindPackage substitution (`#23 <https://github.com/ros2/launch_ros/issues/23>`_)
* Restrict yaml loading in evaluate_parameters (`#33 <https://github.com/ros2/launch_ros/issues/33>`_)
* Fix typo
* Use wildcard syntax in generated parameter YAML files (`#35 <https://github.com/ros2/launch_ros/issues/35>`_)
* Contributors: Dan Rose, Dirk Thomas, Jacob Perron, Michel Hidalgo, Scott K Logan, ivanpauno

0.8.4 (2019-05-30)
------------------
* Update usage of 'noqa' for imports (`#32 <https://github.com/ros2/launch_ros/issues/32>`_)
* Contributors: Jacob Perron

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
