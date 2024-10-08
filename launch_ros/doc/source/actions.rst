`launch_ros` actions
====================

Action frontend arguments names
-------------------------------

Most of the `launch_ros` Python actions (:class:`launch_ros.actions`) are `available through the XML and YAML frontends <https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html>`_.
The name of the action arguments might vary between the underlying Python implementation and the frontends.

:class:`launch_ros.actions.Node`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Action frontend name: ``node``.

.. list-table::
   :header-rows: 1

   * - Python argument name
     - frontend argument name
   * - ``arguments``
     - ``args``
   * - ``executable``
     - ``exec``
   * - ``exec_name``
     - ``exec_name``
   * - ``namespace``
     - ``namespace``
   * - ``node_name``
     - ``node-name``
   * - ``package``
     - ``pkg``
   * - ``parameters``
     - ``param`` (``name``, ``value``)
   * - ``remappings``
     - ``remap`` (``from``, ``to``)
   * - ``ros_arguments``
     - ``ros_args``

:class:`launch_ros.actions.PushROSNamespace`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Action frontend name: ``push_ros_namespace``.

.. list-table::
   :header-rows: 1

   * - Python argument name
     - frontend argument name
   * - ``namespace``
     - ``namespace``
