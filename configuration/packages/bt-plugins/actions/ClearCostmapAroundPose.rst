.. _bt_clear_costmap_around_pose_action:


ClearCostmapAroundPose
======================

Action to call a costmap clearing around a given pose server.

Input Ports
-----------

:pose:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped N/A
  =============================== =======

  Description
    	Pose around which to clear the costmap

:reset_distance:

  ============== =======
  Type           Default
  -------------- -------
  double         1.0
  ============== =======

  Description
    	Distance from the pose under which obstacles are cleared

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

  <ClearCostmapAroundPose name="ClearLocalCostmapAroundPose" 
                          service_name="local_costmap/clear_around_pose_local_costmap" 
                          pose="{goal_pose}" 
                          reset_distance="2.0"/>