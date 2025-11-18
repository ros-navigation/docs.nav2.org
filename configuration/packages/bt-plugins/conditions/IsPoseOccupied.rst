.. _bt_is_pose_occupied_condition:

IsPoseOccupied
==============

Checks to see if the pose is occupied. If it is occupied, the condition returns SUCCESS, otherwise
it returns FAILURE.

Input Ports
-----------

:pose:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A
  ========================= =======

  Description
      Pose to check if it is occupied.

:service_name:

  ====== =======================================
  Type   Default
  ------ ---------------------------------------
  string /global_costmap/get_cost_global_costmap
  ====== =======================================

  Description
    costmap service name responsible for getting the cost.

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

:server_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 20.0
  ====== =======

  Description
    Service response timeout (ms).

:consider_unknown_as_obstacle:

  ====== =======
  Type   Default
  ------ -------
  bool   false
  ====== =======

  Description
    Whether to consider unknown cost (255) as obstacle.


Example
-------

.. code-block:: xml

    <IsPoseOccupied server_timeout="10" pose="{goal}"/>
