.. _bt_compute_path_to_pose_action:

ComputePathToPose
=================

Invokes the ComputePathToPose ROS 2 action server, which is implemented by the nav2_planner_ module.
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
            Start pose. Optional. Used as the planner start pose instead of the current robot pose, if ``use_start`` is not false (i.e. not provided or set to true). Takes in a blackboard variable, e.g. "{start}".

    :use_start:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
            Optional. For using or not using (i.e. ignoring) the provided start pose ``start``. Takes in a blackboard variable, e.g. "{use_start}".

    :goal:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
            Goal pose. Takes in a blackboard variable, e.g. "{goal}".

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

  .. group-tab:: Kilted and newer

    :start:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
            Start pose. Optional. Used as the planner start pose instead of the current robot pose, if ``use_start`` is not false (i.e. not provided or set to true). Takes in a blackboard variable, e.g. "{start}".

    :use_start:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
            Optional. For using or not using (i.e. ignoring) the provided start pose ``start``. Takes in a blackboard variable, e.g. "{use_start}".

    :goal:

      ===================================== =======
      Type                                  Default
      ------------------------------------- -------
      geometry_msgs::msg::PoseStamped         N/A
      ===================================== =======

      Description
            Goal pose. Takes in a blackboard variable, e.g. "{goal}".

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

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        Compute path to pose error code. See ``ComputePathToPose`` action message for the enumerated set of error codes.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Compute path to pose error message. See ``ComputePathToPose`` action message for the enumerated set of error codes.

Example
-------

.. tabs::

  .. group-tab:: Lyrical and newer

    .. code-block:: xml

      <ComputePathToPose goal="{goal}" path="{path}" planner_id="grid_based" server_name="compute_path_to_pose" server_timeout="10"
                        error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>

  .. group-tab:: Kilted and older

    .. code-block:: xml

      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_name="compute_path_to_pose" server_timeout="10"
                        error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
