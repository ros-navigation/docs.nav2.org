.. _bt_compute_route_action:

ComputeRoute
============

Invokes the ComputeRoute ROS 2 action server, which is implemented by the nav2_route_ module.
The server address can be remapped using the ``server_name`` input port.

.. _nav2_route: https://github.com/ros-navigation/navigation2/tree/main/nav2_route

Input Ports
-----------
:start:

  ===================================== =======
  Type                                  Default
  ------------------------------------- -------
  geometry_msgs::msg::PoseStamped         N/A
  ===================================== =======

  Description
        Start pose. Optional. Only used if not left empty. Takes in a blackboard variable, e.g. "{start}".

:goal:

  ===================================== =======
  Type                                  Default
  ------------------------------------- -------
  geometry_msgs::msg::PoseStamped         N/A
  ===================================== =======

  Description
        Goal pose. Takes in a blackboard variable, e.g. "{goal}".

:start_id:

  ===================================== =======
  Type                                  Default
  ------------------------------------- -------
  int                                   N/A
  ===================================== =======

  Description
        Start node ID to use.

:goal_id:

  ===================================== =======
  Type                                  Default
  ------------------------------------- -------
  int                                   N/A
  ===================================== =======

  Description
        Goal node ID to use.

:use_start:

  ============== =======
  Type           Default
  -------------- -------
  bool           false
  ============== =======

  Description
        Whether to use the start or use TF to obtain the robot's start pose.

:use_poses:

  ============== =======
  Type           Default
  -------------- -------
  bool           false
  ============== =======

  Description
        Whether to use the start and goal poses or start and goal node IDs.

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

:route:

  ========================== =======
  Type                       Default
  -------------------------- -------
  nav2_msgs::msg::Route      N/A
  ========================== =======

  Description
        Route created by action server. Takes in a blackboard variable, e.g. "{route}".

:path:

  ========================== =======
  Type                       Default
  -------------------------- -------
  nav_msgs::msg::Path         N/A
  ========================== =======

  Description
        Path created by action server. Takes in a blackboard variable, e.g. "{path}".

:planning_time:

  ================================= =======
  Type                              Default
  --------------------------------- -------
  builtin_interfaces::msg::Duration N/A
  ================================= =======

  Description
        Time it took to compute the route.

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        Compute route error code. See ``ComputeRoute`` action message for the enumerated set of error codes.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Compute route error message. See ``ComputeRoute`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <ComputeRoute start="{start}" goal="{goal}" use_poses="{true}" use_start="{true}" path="{path}" server_name="compute_route" server_timeout="10"
                     error_code_id="{compute_route_error_code}" error_msg="{compute_route_error_msg}"/>
