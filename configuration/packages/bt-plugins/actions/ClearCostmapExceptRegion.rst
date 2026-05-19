.. _bt_clear_costmap_except_region_action:


ClearCostmapExceptRegion
========================

Action to call a costmap clearing except region server.

Input Ports
-----------

:reset_distance:

  ============== =======
  Type           Default
  -------------- -------
  double         1
  ============== =======

  Description
      side size of the square area centered on the robot that will not be cleared on the costmap (all the rest of the costmap will)

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

:plugins:

  ===================== =======
  Type                  Default
  --------------------- -------
  std::vector<string>   N/A
  ===================== =======

  Description
      Optional. A list of costmap plugin names to be cleared.
      If specified, only these costmap plugins will be cleared.
      Otherwise, all "clearable" costmap plugins will be cleared.

Example
-------

.. code-block:: xml

  <ClearCostmapExceptRegion name="ClearLocalCostmap-Subtree"
                            service_name="local_costmap/clear_except_local_costmap"
                            reset_distance="2.0"
                            plugins="obstacle_layer;voxel_layer"/>
