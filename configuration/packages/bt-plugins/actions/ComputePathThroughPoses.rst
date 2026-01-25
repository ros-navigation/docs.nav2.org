.. _bt_compute_path_through_poses_action:

ComputePathThroughPoses
=======================

Invokes the ComputePathThroughPoses ROS 2 action server, which is implemented by the nav2_planner_ module.
The server address can be remapped using the ``server_name`` input port.

.. _nav2_planner: https://github.com/ros-navigation/navigation2/tree/main/nav2_planner

Input Ports
-----------

.. tabs::

  .. group-tab:: Lyrical and newer

    :start:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
          Start pose. Optional. Only used if not left empty. Takes in a blackboard variable, e.g. "{start}".

    :goals:

      ==================== =======
      Type                 Default
      -------------------- -------
      nav_msgs::msg::Goals   N/A
      ==================== =======

      Description
          Goal poses. Takes in a blackboard variable, e.g. "{goals}".

    :planner_id:

      ============== =======
      Type           Default
      -------------- -------
      string         N/A
      ============== =======

      Description
          Mapped name to the planner plugin type to use, e.g. grid_based.

    :server_name:

      ============== =======
      Type           Default
      -------------- -------
      string         N/A
      ============== =======

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

    :start:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
          Start pose. Optional. Only used if not left empty. Takes in a blackboard variable, e.g. "{start}".

    :goals:

      ==================== =======
      Type                 Default
      -------------------- -------
      nav_msgs::msg::Goals   N/A
      ==================== =======

      Description
          Goal poses. Takes in a blackboard variable, e.g. "{goals}".

    :planner_id:

      ============== =======
      Type           Default
      -------------- -------
      string         N/A
      ============== =======

      Description
          Mapped name to the planner plugin type to use, e.g. GridBased.

    :server_name:

      ============== =======
      Type           Default
      -------------- -------
      string         N/A
      ============== =======

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

:path:

  ========================== =======
  Type                       Default
  -------------------------- -------
  nav_msgs::msg::Path         N/A
  ========================== =======

  Description
      Path created by action server. Takes in a blackboard variable, e.g. "{path}".

:last_reached_index:

  ========================== =======
  Type                       Default
  -------------------------- -------
  int16                       -1
  ========================== =======

  Description
      In the case of a partial plan, index of the last reached pose from the goals list. Otherwise -1 which also corresponds to ComputePathThroughPosesResult::ALL_GOALS if a full plan through all the goals was possible.

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
      Compute path through poses error code. See ``ComputePathThroughPoses`` action message for the enumerated set of error codes.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
      Compute path through poses error message. See ``ComputePathThroughPoses`` action message for the enumerated set of error codes.

Example
-------

.. tabs::

  .. group-tab:: Lyrical and newer

    .. code-block:: xml

      <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="grid_based" server_name="compute_path_through_poses" server_timeout="10" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>

  .. group-tab:: Kilted and older

    .. code-block:: xml

      <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased" server_name="compute_path_through_poses" server_timeout="10" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
