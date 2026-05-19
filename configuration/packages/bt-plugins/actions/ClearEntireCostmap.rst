.. _bt_clear_entire_costmap_action:


ClearEntireCostmap
==================

Action to call a costmap clearing server.

Input Ports
-----------

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
      If specified, only these costmap plugins and the master costmap will be cleared.
      Otherwise, the entire costmap will be cleared.

Example
-------

.. code-block:: xml

  <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
                      service_name="local_costmap/clear_entirely_local_costmap"
                      plugins="obstacle_layer;voxel_layer"/>
