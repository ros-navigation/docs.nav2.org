.. _bt_follow_path_action:

FollowPath
==========

Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

.. tabs::

  .. group-tab:: Lyrical and newer

    :path:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Takes in a blackboard variable containing the path to follow, eg. "{path}".

    :controller_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Mapped name of the controller plugin type to use, e.g. follow_path.

    :goal_checker_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Mapped name of the goal checker plugin type to use, e.g. simple_goal_checker.

    :progress_checker_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
          Mapped name of the progress checker plugin type to use, e.g. simple_progress_checker.

    :path_handler_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
          Mapped name of the path handler plugin type to use, e.g. feasible_path_handler.

    :server_name:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Action server name.


    :server_timeout:

      ============== =======
      Type           Default
      -------------- -------
      double         10
      ============== =======

      Description
            Action server timeout (ms).

  .. group-tab:: Kilted and older

    :path:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Takes in a blackboard variable containing the path to follow, eg. "{path}".

    :controller_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Mapped name of the controller plugin type to use, e.g. FollowPath.

    :goal_checker_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Mapped name of the goal checker plugin type to use, e.g. SimpleGoalChecker.

    :progress_checker_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
          Mapped name of the progress checker plugin type to use, e.g. SimpleProgressChecker.

    :path_handler_id:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
          Mapped name of the path handler plugin type to use, e.g. FeasiblePathHandler.

    :server_name:

      ====== =======
      Type   Default
      ------ -------
      string N/A
      ====== =======

      Description
            Action server name.


    :server_timeout:

      ============== =======
      Type           Default
      -------------- -------
      double         10
      ============== =======

      Description
            Action server timeout (ms).

Output Ports
------------

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        Follow path error code. See ``FollowPath`` action for the enumerated set of error code definitions.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Follow path error message. See ``FollowPath`` action for the enumerated set of error code definitions.

:tracking_feedback:

  ================================ =======
  Type                             Default
  -------------------------------- -------
  nav2_msgs::msg::TrackingFeedback N/A
  ================================ =======

  Description
        Tracking feedback message from the controller server, including cross track error, current path index, remaining path length, etc.



Example
-------

.. tabs::

  .. group-tab:: Lyrical and newer

    .. code-block:: xml

      <FollowPath path="{path}" controller_id="follow_path" goal_checker_id="precise_goal_checker" path_handler_id="path_handler" server_name="follow_path" server_timeout="10" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}" tracking_feedback="{tracking_feedback}"/>

  .. group-tab:: Kilted and older

    .. code-block:: xml

      <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" path_handler_id="PathHandler" server_name="FollowPath" server_timeout="10" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}" tracking_feedback="{tracking_feedback}"/>
