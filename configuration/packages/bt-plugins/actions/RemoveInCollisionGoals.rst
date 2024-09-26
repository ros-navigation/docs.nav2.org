.. _bt_remove_in_collision_goals_action:

RemoveInCollisionGoals
======================

Looks over the input port ``goals`` and removes any waypoint that has a point or footprint cost above a certain threshold.
This may be used to cull goal points passed from ``ComputePathThroughPoses`` to avoid waiting indefinitely on occupied waypoints.

Input Ports
-----------

:service_name:

  ====== =======================================
  Type   Default
  ------ ---------------------------------------
  string /global_costmap/get_cost_global_costmap  
  ====== =======================================

  Description
    costmap service name responsible for getting the cost.

:input_goals:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped   N/A  
  =============================== =======

  Description
    A vector of goals to check if in collision

:cost_threshold:

  ====== =======
  Type   Default
  ------ -------
  double 254.0  
  ====== =======

  Description
    The cost threshold above which a waypoint is considered in collision and should be removed. If ``use_footprint = false``, consider setting to 253 for occupied. 

:use_footprint:

  ====== =======
  Type   Default
  ------ -------
  bool   true  
  ====== =======

  Description
    Whether to use the footprint cost or the point cost.

:consider_unknown_as_obstacle:

  ====== =======
  Type   Default
  ------ -------
  bool   false  
  ====== =======

  Description
    Whether to consider unknown cost (255) as obstacle.

Output Ports
------------

:output_goals:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped   N/A  
  =============================== =======

  Description
    A vector of goals containing only those that are not in collision.

Example
-------

.. code-block:: xml

  <RemoveInCollisionGoals input_goals="{goals}" output_goals="{goals}" cost_threshold="254.0" use_footprint="true" service_name="/global_costmap/get_cost_global_costmap" />
    
