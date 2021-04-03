.. _bt_clear_entire_costmap_around_robot_action:


ClearCostmapAroundRobot
=======================

Action to call a costmap clearing around robot server.

Input Ports
-----------

:reset_distance:

  ============== =======
  Type           Default
  -------------- -------
  double         1  
  ============== =======

  Description
    	side size of the square area centered on the robot that will be cleared on the costmap (the rest of the costmap won't)
    	
:service_name:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	costmap service name responsible for clearing the costmap.

:server_timeout:

  ============== =======
  Type           Default
  -------------- -------
  double         10  
  ============== =======

  Description
    	Action server timeout (ms).

Example
-------

.. code-block:: xml

  <ClearCostmapAroundRobot name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_around_local_costmap"/>
