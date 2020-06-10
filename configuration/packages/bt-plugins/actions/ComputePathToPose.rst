.. bt_actions:

ComputePathToPose
=================

Invokes the ComputePathToPose ROS2 action server, which is implemented by the nav2_planner_ module. 
The server address can be remapped using the ``server_name`` input port.

.. _nav2_planner: https://github.com/ros-planning/navigation2/tree/master/nav2_planner

Input Ports
-----------

:goal:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Goal pose.

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

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Path created by action server.

Example
-------

.. code-block:: xml

  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_name="ComputePathToPose_server" server_timeout="10"/>
