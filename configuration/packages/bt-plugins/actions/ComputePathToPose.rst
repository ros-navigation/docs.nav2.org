.. _bt_compute_path_to_pose_action:

ComputePathToPose
=================

Invokes the ComputePathToPose ROS 2 action server, which is implemented by the nav2_planner_ module. 
The server address can be remapped using the ``server_name`` input port.

.. _nav2_planner: https://github.com/ros-navigation/navigation2/tree/main/nav2_planner

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

Example
-------

.. code-block:: xml

  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_name="ComputePathToPose" server_timeout="10" error_code_id="{compute_path_error_code}"/>
