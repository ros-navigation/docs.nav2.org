.. _bt_is_path_valid_condition:

IsPathValid
===========

Checks to see if the global path is valid. If there is an
obstacle along the path, the condition returns FAILURE, otherwise
it returns SUCCESS. Optionally checks specific costmap layers and
can use a custom footprint for validation.

Input Ports
-----------

:server_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 20.0
  ====== =======

  Description
    Service response timeout (ms).

:path:

  ==================================== =======
  Type                                 Default
  ------------------------------------ -------
  nav_msgs::msg::Path                  N/A
  ==================================== =======

  Description
    The global path to check for validity.

:max_cost:

  ============== ==========
  Type           Default
  -------------- ----------
  unsigned int   254
  ============== ==========

  Description
    The maximum allowable cost for the path to be considered valid.

:consider_unknown_as_obstacle:

  ====== =======
  Type   Default
  ------ -------
  bool   false
  ====== =======

  Description
    Whether to consider unknown cost (255) as obstacle.

:layer_name:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    Name of the specific costmap layer to check against.
    If empty, checks against the full costmap.

:footprint:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    Custom footprint specification as a bracketed array of arrays,
    e.g., "[[x1,y1],[x2,y2],...]". If empty, uses the robot's
    configured footprint.

:check_full_path:

  ====== =======
  Type   Default
  ------ -------
  bool   false
  ====== =======

  Description
    Whether to check all poses in the path (true) or stop at the
    first invalid pose (false). When true, all collision poses
    are reported.

Output Ports
------------

:collision_poses:

  ============================================ =======
  Type                                         Default
  -------------------------------------------- -------
  std::vector<geometry_msgs::msg::PoseStamped> N/A
  ============================================ =======

  Description
    Vector of poses in the path that are in collision or invalid.
    Empty if the path is valid.


Example
-------

.. code-block:: xml

    <IsPathValid
      server_timeout="10"
      path="{path}"
      max_cost="100"
      consider_unknown_as_obstacle="false"
      layer_name=""
      footprint=""
      check_full_path="false"
      collision_poses="{collision_poses}" />

With custom footprint:

.. code-block:: xml

    <IsPathValid
      path="{path}"
      footprint="[[0.5,0.5],[0.5,-0.5],[-0.5,-0.5],[-0.5,0.5]]"
      collision_poses="{collision_poses}" />

Checking a specific costmap layer:

.. code-block:: xml

    <IsPathValid
      path="{path}"
      layer_name="obstacle_layer"
      check_full_path="true"
      collision_poses="{collision_poses}" />
