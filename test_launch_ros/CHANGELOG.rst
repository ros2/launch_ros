^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_launch_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.2 (2020-05-26)
-------------------

0.10.1 (2020-05-13)
-------------------
* Clean up various pytest warnings (`#143 <https://github.com/ros2/launch_ros/issues/143>`_)
* Contributors: Michael Carroll

0.10.0 (2020-04-29)
-------------------
* Deprecated 'node_executable' parameter and replace with 'executable' (`#140 <https://github.com/ros2/launch_ros/issues/140>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Enable implicit ROS startup by launch_ros actions.  (`#128 <https://github.com/ros2/launch_ros/issues/128>`_)
* Fix flake8 linter errors (`#130 <https://github.com/ros2/launch_ros/issues/130>`_)
* Add warning message when launching Non-Uniquely Named Nodes (`#127 <https://github.com/ros2/launch_ros/issues/127>`_)
* Rename node-related parameters (`#122 <https://github.com/ros2/launch_ros/issues/122>`_)
* Fix frontend topic remapping (`#111 <https://github.com/ros2/launch_ros/issues/111>`_)
* Maintain order of parameters regarding name and from (`#99 <https://github.com/ros2/launch_ros/issues/99>`_)
* Fix push-ros-namespace in xml/yaml launch files (`#100 <https://github.com/ros2/launch_ros/issues/100>`_)
* Contributors: Brian Marchi, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Miaofei Mei, Michel Hidalgo

0.9.4 (2019-11-19)
------------------

0.9.3 (2019-11-13)
------------------
* Install package.xml (`#92 <https://github.com/ros2/launch_ros/issues/92>`_)
* Contributors: Gaël Écorchard

0.9.2 (2019-10-23)
------------------
* Fix launch_ros.actions.Node parsing function (`#83 <https://github.com/ros2/launch_ros/issues/83>`_)
* Contributors: Michel Hidalgo

0.9.1 (2019-09-28)
------------------

0.9.0 (2019-09-25)
------------------
* Handle zero-width string parameters. (`#72 <https://github.com/ros2/launch_ros/issues/72>`_)
* Add substitution for finding package share directory (`#57 <https://github.com/ros2/launch_ros/issues/57>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#52 <https://github.com/ros2/launch_ros/issues/52>`_)
* Use node namespace if no other was specified (`#51 <https://github.com/ros2/launch_ros/issues/51>`_)
* [launch frontend] Rename some tag attributes (`#47 <https://github.com/ros2/launch_ros/issues/47>`_)
* Fix PushRosNamespace action (`#44 <https://github.com/ros2/launch_ros/issues/44>`_)
* Add PushRosNamespace action (`#42 <https://github.com/ros2/launch_ros/issues/42>`_)
* Add frontend parsing methods for Node, ExecutableInPackage and FindPackage substitution (`#23 <https://github.com/ros2/launch_ros/issues/23>`_)
* Restrict yaml loading in evaluate_parameters (`#33 <https://github.com/ros2/launch_ros/issues/33>`_)
* Use wildcard syntax in generated parameter YAML files (`#35 <https://github.com/ros2/launch_ros/issues/35>`_)
* Contributors: Jacob Perron, Michel Hidalgo, Scott K Logan, ivanpauno

0.8.4 (2019-05-30)
------------------

0.8.3 (2019-05-29)
------------------
* Added the ``FindPackage`` substitution. (`#22 <https://github.com/ros2/launch_ros/issues/22>`_)
* Changed interpretation of Parameter values which are passed to ``Node()`` so that they get evaluated by yaml rules. (`#31 <https://github.com/ros2/launch_ros/issues/31>`_)
* Contributors: Shane Loretz, ivanpauno

0.8.2 (2019-05-20)
------------------

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-14)
------------------
* Added normalize_parameters and evaluate_paramters. (`#192 <https://github.com/ros2/launch/issues/192>`_)
* Added normalize_remap_rule and types. (`#173 <https://github.com/ros2/launch/issues/173>`_)
* Added support for required nodes. (`#179 <https://github.com/ros2/launch/issues/179>`_)
* Contributors: Kyle Fazzari, Shane Loretz

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
* Fixed a bug to ensure that shutdown event is handled correctly (`#154 <https://github.com/ros2/launch/issues/154>`_)
  * There was a potential race condition in between when the shutdown event is emitted and the rest of the shutdown handling code.
  * This introduces an additional await to ensure that the event is emitted before proceeding.
* Added support for passing parameters as a dictionary to a Node (`#138 <https://github.com/ros2/launch/issues/138>`_)
* Made various fixes and added tests for remappings passed to Node actions (`#137 <https://github.com/ros2/launch/issues/137>`_)
* Added ability to pass parameter files to Node actions (`#135 <https://github.com/ros2/launch/issues/135>`_)
* Contributors: Michael Carroll, dhood
